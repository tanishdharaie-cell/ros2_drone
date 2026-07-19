#!/usr/bin/env python3
"""
yolo_search_controller.py 

Terminal prompts for:
  1. What to search for (any YOLO class: person, car, dog, chair, etc.)

Drone then:
  - Takes off, climbs to 3m
  - Flies expanding spiral search pattern
  - Runs YOLO at ~2 FPS on BOTH cameras (low compute)
  - When target detected → flies TO it
  - Hovers over it → outputs AEROPIN of detected object
  - Lands

No payload. No separate detector node needed — YOLO runs inside this node.

"""

import math
import os
import sys
import time
import subprocess

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, String, Int8, Bool
from ros_gz_interfaces.msg import Contacts
from cv_bridge import CvBridge

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
from aeropin import encode as aeropin_encode

# ── Plugin states ────────────────────────────────────────────────────
LANDED_MODEL = 0
FLYING_MODEL = 2

# ── Mission states ───────────────────────────────────────────────────
IDLE     = 'idle'
TAKEOFF  = 'takeoff'
CLIMB    = 'climb'
SEARCH   = 'search'
APPROACH = 'approach'
HOVER    = 'hover'
LAND     = 'land'

# ── Camera geometry (from URDF) ──────────────────────────────────────
BOTTOM_FOV_H = 1.047;  BOTTOM_IMG_W = 640;  BOTTOM_IMG_H = 360
FRONT_FOV_H  = 2.09;   FRONT_IMG_W  = 640;  FRONT_IMG_H  = 360
FRONT_CAM_X  = 0.2     # front cam offset from base_link

# COCO classes for reference
COCO_CLASSES = [
    'person','bicycle','car','motorcycle','airplane','bus','train','truck',
    'boat','traffic light','fire hydrant','stop sign','parking meter','bench',
    'bird','cat','dog','horse','sheep','cow','elephant','bear','zebra',
    'giraffe','backpack','umbrella','handbag','tie','suitcase','frisbee',
    'skis','snowboard','sports ball','kite','baseball bat','baseball glove',
    'skateboard','surfboard','tennis racket','bottle','wine glass','cup',
    'fork','knife','spoon','bowl','banana','apple','sandwich','orange',
    'broccoli','carrot','hot dog','pizza','donut','cake','chair','couch',
    'potted plant','bed','dining table','toilet','tv','laptop','mouse',
    'remote','keyboard','cell phone','microwave','oven','toaster','sink',
    'refrigerator','book','clock','vase','scissors','teddy bear',
    'hair drier','toothbrush'
]


def _prompt_target() -> str:
    """Ask user what to search for."""
    print()
    print('╔══════════════════════════════════════════════════════╗')
    print('║    ROS2 Drone — OBJECT SEARCH + AEROPIN Mode         ║')
    print('║    No payload. YOLO detection on both cameras.       ║')
    print('╠══════════════════════════════════════════════════════╣')
    print('║  Available YOLO classes (examples):                  ║')
    print('║    person, car, dog, cat, bicycle, truck, bus,       ║')
    print('║    chair, couch, bottle, cup, backpack, umbrella,    ║')
    print('║    fire hydrant, bench, potted plant, dining table   ║')
    print('╚══════════════════════════════════════════════════════╝')
    print()

    while True:
        raw = input('  Search for > ').strip().lower()
        if not raw:
            continue
        if raw in COCO_CLASSES:
            print(f'  ✓  Target class: "{raw}"')
            return raw
        # fuzzy match
        matches = [c for c in COCO_CLASSES if raw in c or c in raw]
        if matches:
            print(f'  ≈  Did you mean: {", ".join(matches)}')
            if len(matches) == 1:
                print(f'  ✓  Using: "{matches[0]}"')
                return matches[0]
        else:
            print(f'  ✗  "{raw}" not in YOLO classes. Try again.')


def _ensure_yolo():
    """Install ultralytics if needed."""
    try:
        import ultralytics
        return True
    except ImportError:
        print('  Installing ultralytics...')
        subprocess.call([sys.executable, '-m', 'pip', 'install',
                         'ultralytics', 'opencv-python',
                         '--break-system-packages', '-q'])
        return True


class ObjectSearchController(Node):

    def __init__(self, target_class: str):
        super().__init__('yolo_search_controller')

        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.declare_parameter('drone_namespace', 'simple_drone')
        self.declare_parameter('cruise_alt',      3.0)
        self.declare_parameter('search_radius',   8.0)
        self.declare_parameter('search_speed',    0.5)
        self.declare_parameter('inference_fps',    2.0)  # low FPS

        ns                = self.get_parameter('drone_namespace').value
        self.cruise_alt   = self.get_parameter('cruise_alt').value
        self.search_r     = self.get_parameter('search_radius').value
        self.search_speed = self.get_parameter('search_speed').value
        inf_fps           = self.get_parameter('inference_fps').value

        self.target_class = target_class
        self.confidence   = 0.65

        # ── Load YOLO model ──────────────────────────────────────────
        from ultralytics import YOLO
        self.get_logger().info('Loading YOLOv8n...')
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info(f'YOLO ready — searching for: "{target_class}"')
        self.bridge = CvBridge()

        # ── State ────────────────────────────────────────────────────
        self.state       = IDLE
        self.px = self.py = self.pz = 0.0
        self.yaw = self.vz = 0.0
        self.drone_state = LANDED_MODEL
        self.collision_detected = False
        self.retreat_until = None
        self.takeoff_sent_t = None
        self.land_t      = None

        # detection result
        self.found        = False
        self.target_world_x = 0.0
        self.target_world_y = 0.0
        self.target_pin     = ''
        self.hover_start    = None

        # search pattern
        self.waypoints = self._gen_spiral()
        self.wp_idx    = 0
        self.origin_x = self.origin_y = 0.0

        # frame throttle — process every Nth frame
        self.inf_interval = 1.0 / inf_fps
        self.last_front_t = 0.0
        self.last_bottom_t = 0.0

        # ── Pubs ─────────────────────────────────────────────────────
        self.cmd_pub     = self.create_publisher(Twist,  f'/{ns}/cmd_vel',  10)
        self.takeoff_pub = self.create_publisher(Empty,  f'/{ns}/takeoff',  10)
        self.land_pub    = self.create_publisher(Empty,  f'/{ns}/land',     10)
        self.posctrl_pub = self.create_publisher(Bool,   f'/{ns}/posctrl',  10)

        # ── Subs ─────────────────────────────────────────────────────
        self.create_subscription(Odometry, f'/{ns}/odom',  self._odom_cb,  10)
        self.create_subscription(Int8,     f'/{ns}/state', self._state_cb, 10)
        self.create_subscription(
            Contacts, f'/{ns}/collision', self._collision_cb, 10)
        self.create_subscription(
            Image, f'/{ns}/front/image_raw',  self._front_img_cb,  10)
        self.create_subscription(
            Image, f'/{ns}/bottom/image_raw', self._bottom_img_cb, 10)

        self.create_timer(0.1, self._loop)

        self.get_logger().info(
            f'Search: "{target_class}"  cruise={self.cruise_alt}m  '
            f'YOLO@{inf_fps:.0f}fps  spiral={len(self.waypoints)} waypoints')
        self._start_timer = self.create_timer(2.0, self._auto_start)

    # ── Spiral pattern ───────────────────────────────────────────────

    def _gen_spiral(self):
        # ==========================================================
        # TODO 1 — Generate an expanding square-spiral search pattern
        #
        # Build a list of (x, y) waypoints, relative to the takeoff
        # point, that spiral outward in expanding square "rings"
        # until they leave the search radius.
        #
        # Requirements:
        # - Start at (0, 0) and move in steps of size `step`.
        # - The direction cycles through: +x, +y, -x, -y (right,
        #   up, left, down) — a standard square spiral.
        # - The number of steps taken in a given direction before
        #   turning increases every two direction changes (1 step,
        #   1 step, 2 steps, 2 steps, 3 steps, 3 steps, ...) — this
        #   is what makes the square expand outward instead of
        #   looping in place.
        # - Stop adding waypoints once a point falls outside
        #   [-search_r, search_r] in either x or y, and return the
        #   list collected so far.
        #
        # Hint:
        # Use:
        #   • self.search_r
        #   • A direction index 0..3 mapped to (dx, dy) pairs:
        #       0 → (step, 0)   1 → (0, step)
        #       2 → (-step, 0)  3 → (0, -step)
        #   • Two counters: one for steps taken in the current leg,
        #     one for how many legs completed at the current length
        #     (increase leg length every 2 completed legs)
        # ==========================================================

        # YOUR CODE HERE
        step = self.search_speed * 2.0
        waypoints = []
        x, y = 0.0, 0.0
        dirs = [(step, 0.0), (0.0, step), (-step, 0.0), (0.0, -step)]
        dir_idx = 0
        leg_len = 1
        legs_at_this_len = 0

        while True:
            dx, dy = dirs[dir_idx]
            for _ in range(leg_len):
                x += dx
                y += dy
                if abs(x) > self.search_r or abs(y) > self.search_r:
                    return waypoints
                waypoints.append((x, y))
            dir_idx = (dir_idx + 1) % 4
            legs_at_this_len += 1
            if legs_at_this_len == 2:
                leg_len += 1
                legs_at_this_len = 0

    # ── Sensor callbacks ─────────────────────────────────────────────

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self.px, self.py, self.pz = p.x, p.y, p.z
        self.vz = msg.twist.twist.linear.z
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y ** 2 + q.z ** 2))

    def _state_cb(self, msg):
        self.drone_state = msg.data

    def _collision_cb(self, msg: Contacts):
        if len(msg.contacts) > 0:
            if not self.collision_detected:
                self.get_logger().warn('⚠️  Collision detected — retreating')
            self.collision_detected = True
            self.retreat_until = time.time() + 1.5

    # ── Camera YOLO callbacks (throttled) ────────────────────────────

    def _front_img_cb(self, msg):
        if self.state not in (SEARCH, APPROACH) or self.found:
            return
        now = time.time()
        if now - self.last_front_t < self.inf_interval:
            return
        self.last_front_t = now
        self._run_yolo(msg, 'front')

    def _bottom_img_cb(self, msg):
        if self.state not in (SEARCH, APPROACH) or self.found:
            return
        now = time.time()
        if now - self.last_bottom_t < self.inf_interval:
            return
        self.last_bottom_t = now
        self._run_yolo(msg, 'bottom')

    def _run_yolo(self, msg, cam: str):
        """Run inference, if target found estimate world position."""
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(img, conf=self.confidence, verbose=False)

            for r in results:
                for box in r.boxes:
                    cls = self.model.names[int(box.cls[0])].lower()
                    if cls != self.target_class:
                        continue
                    conf = float(box.conf[0])
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    cx = (x1 + x2) / 2.0
                    cy = (y1 + y2) / 2.0
                    bw = x2 - x1
                    bh = y2 - y1

                    wx, wy = self._estimate_world_pos(
                        cam, cx, cy, bw, bh)

                    if wx is None:
                        continue

                    try:
                        pin = aeropin_encode(wx, wy)
                    except ValueError:
                        continue

                    self.target_world_x = wx
                    self.target_world_y = wy
                    self.target_pin     = pin
                    self.found          = True

                    self.get_logger().info(
                        f'🎯 FOUND "{self.target_class}" [{cam.upper()}]  '
                        f'pos=({wx:.2f}, {wy:.2f}) m  '
                        f'AEROPIN: {pin}  conf={conf:.0%}')
                    self.get_logger().info(
                        f'Flying to target...')
                    self._go(APPROACH)
                    return

        except Exception as e:
            self.get_logger().error(f'YOLO [{cam}]: {e}', throttle_duration_sec=5.0)

    def _estimate_world_pos(self, cam, cx, cy, bw, bh):
        """Estimate world XY from detection bbox + drone odom."""
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)

        if cam == 'bottom':
            # ==========================================================
            # TODO 2a — Estimate target world position from the
            # bottom (downward-facing) camera
            #
            # The bottom camera looks straight down, so a detection
            # offset from the image centre corresponds to a real
            # ground offset, scaled by altitude.
            #
            # Requirements:
            # - If altitude (self.pz) is below 0.5 m, return
            #   (None, None) — too close to the ground to trust this
            #   projection.
            # - Normalize the detection centre (cx, cy) to [-0.5, 0.5]
            #   relative to image width/height (BOTTOM_IMG_W/H).
            # - Compute the vertical FOV from the horizontal FOV and
            #   image aspect ratio.
            # - Convert each normalized offset to a body-frame ground
            #   offset using altitude and tan(half-angle):
            #     offset = altitude * tan(normalized_offset * FOV)
            # - Rotate that body-frame offset into world coordinates
            #   using the drone's yaw (cos_y, sin_y), and add the
            #   drone's current world position (self.px, self.py).
            # - Return (world_x, world_y).
            #
            # Hint:
            # Use:
            #   • BOTTOM_IMG_W, BOTTOM_IMG_H, BOTTOM_FOV_H
            #   • self.pz, cos_y, sin_y, self.px, self.py
            #   • math.tan
            # ==========================================================

            # YOUR CODE HERE
            if self.pz < 0.5:
                return None, None

            nx = (cx / BOTTOM_IMG_W) - 0.5
            ny = (cy / BOTTOM_IMG_H) - 0.5

            fov_v = BOTTOM_FOV_H * (BOTTOM_IMG_H / BOTTOM_IMG_W)

            body_x = self.pz * math.tan(ny * fov_v)
            body_y = self.pz * math.tan(nx * BOTTOM_FOV_H)

            world_x = self.px + body_x * cos_y - body_y * sin_y
            world_y = self.py + body_x * sin_y + body_y * cos_y

            return world_x, world_y

        elif cam == 'front':
            # ==========================================================
            # TODO 2b — Estimate target world position from the
            # front (forward-facing) camera
            #
            # The front camera looks horizontally, so distance can't
            # be read directly from pixel offset — it has to be
            # estimated from how large the detected object appears
            # (smaller box height ⇒ farther away).
            #
            # Requirements:
            # - If the detected box height (bh) is below 15 px,
            #   return (None, None) — too small/unreliable to
            #   estimate distance from.
            # - Assume a generic real-world object height
            #   (assumed_h = 1.0 m) and compute the camera's focal
            #   length in pixels from FRONT_IMG_H and the vertical
            #   FOV.
            # - Estimate distance using the pinhole projection
            #   relationship: dist = (assumed_h * focal) / bh.
            #   Clamp the result to a reasonable max (e.g. 15 m).
            # - Normalize the horizontal detection centre (cx) to
            #   get the bearing angle within FRONT_FOV_H.
            # - Convert distance + bearing angle into a body-frame
            #   (x, y) offset (forward/sideways), accounting for the
            #   camera's forward offset from base_link (FRONT_CAM_X).
            # - Rotate that body-frame offset into world coordinates
            #   using the drone's yaw, and add the drone's current
            #   world position.
            # - Return (world_x, world_y).
            #
            # Hint:
            # Use:
            #   • FRONT_IMG_W, FRONT_IMG_H, FRONT_FOV_H, FRONT_CAM_X
            #   • bh, cx, cos_y, sin_y, self.px, self.py
            #   • math.tan, math.cos, math.sin
            # ==========================================================

            # YOUR CODE HERE
            if bh < 15:
                return None, None

            assumed_h = 1.0
            focal_px = (FRONT_IMG_H / 2.0) / math.tan(FRONT_FOV_H / 2.0)
            dist = min((assumed_h * focal_px) / bh, 15.0)

            nx = (cx / FRONT_IMG_W) - 0.5
            bearing = nx * FRONT_FOV_H

            body_x = FRONT_CAM_X + dist * math.cos(bearing)
            body_y = dist * math.sin(bearing)

            world_x = self.px + body_x * cos_y - body_y * sin_y
            world_y = self.py + body_x * sin_y + body_y * cos_y

            return world_x, world_y

        return None, None

    # ── Control loop ─────────────────────────────────────────────────

    def _auto_start(self):
        if self.state != IDLE:
            return
        self._start_timer.cancel()
        self.get_logger().info(f'═══ SEARCH MISSION: "{self.target_class}" ═══')
        b = Bool(); b.data = False
        self.posctrl_pub.publish(b)
        self.takeoff_pub.publish(Empty())
        self.takeoff_sent_t = time.time()
        self._go(TAKEOFF)

    def _loop(self):
        {TAKEOFF:  self._do_takeoff,
         CLIMB:    self._do_climb,
         SEARCH:   self._do_search,
         APPROACH: self._do_approach,
         HOVER:    self._do_hover,
         LAND:     self._do_land,
         }.get(self.state, lambda: None)()

    # ── TAKEOFF ──────────────────────────────────────────────────────

    def _do_takeoff(self):
        self.get_logger().info(
            f'TAKEOFF  drone_state={self.drone_state}',
            throttle_duration_sec=1.0)
        if self.drone_state == FLYING_MODEL:
            self.origin_x = self.px  # spiral is centered here, not (0,0)
            self.origin_y = self.py
            self._go(CLIMB); return
        if self.takeoff_sent_t and time.time() - self.takeoff_sent_t > 5.0:
            self.takeoff_pub.publish(Empty())
            self.takeoff_sent_t = time.time()

    # ── CLIMB ────────────────────────────────────────────────────────

    def _do_climb(self):
        err = self.cruise_alt - self.pz
        self.get_logger().info(
            f'CLIMB  z={self.pz:.2f}/{self.cruise_alt:.1f} m',
            throttle_duration_sec=1.0)
        if err < 0.10:
            self._stop(); self._go(SEARCH)
            self.get_logger().info(
                f'Scanning for "{self.target_class}" — '
                f'{len(self.waypoints)} waypoints')
            return
        t = Twist()
        t.linear.z = min(0.4, max(0.05, err * 0.4))
        self.cmd_pub.publish(t)

    # ── SEARCH: fly spiral, YOLO runs in camera callbacks ────────────

    def _do_search(self):
        if self.collision_detected:
            if self.retreat_until and time.time() < self.retreat_until:
                t = Twist()
                t.linear.x = -0.4
                t.linear.z = 0.3
                self.cmd_pub.publish(t)
                return
            else:
                self.collision_detected = False
                self.retreat_until = None
        if self.found:
            return  # detection callback already switched to APPROACH

        if self.wp_idx >= len(self.waypoints):
            self.get_logger().info(
                f'Search complete — "{self.target_class}" NOT FOUND')
            self._stop()
            self.land_pub.publish(Empty())
            self.land_t = time.time()
            self._go(LAND)
            return

        wx, wy = self.waypoints[self.wp_idx]
        tx, ty = self.origin_x + wx, self.origin_y + wy  # spiral is relative to takeoff point
        dx = tx - self.px
        dy = ty - self.py
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < 0.3:
            self.wp_idx += 1
            return

        target_yaw = math.atan2(dy, dx)
        yaw_err = target_yaw - self.yaw
        if yaw_err >  math.pi: yaw_err -= 2 * math.pi
        if yaw_err < -math.pi: yaw_err += 2 * math.pi

        self.get_logger().info(
            f'SEARCH  wp={self.wp_idx + 1}/{len(self.waypoints)}  '
            f'z={self.pz:.2f} m  looking for: {self.target_class}',
            throttle_duration_sec=2.0)

        align = max(0.0, math.cos(yaw_err))  # don't charge forward mid-turn
        t = Twist()
        t.linear.x  = min(self.search_speed, dist * 0.4) * align
        t.angular.z = yaw_err * 1.5
        alt_err     = self.cruise_alt - self.pz
        t.linear.z  = max(-0.5, min(0.5, alt_err * 1.5))
        self.cmd_pub.publish(t)

    # ── APPROACH: fly to detected object ─────────────────────────────

    def _do_approach(self):
        if self.collision_detected:
            if self.retreat_until and time.time() < self.retreat_until:
                t = Twist()
                t.linear.x = -0.4
                self.cmd_pub.publish(t)
                return
            else:
                self.collision_detected = False
                self.retreat_until = None
        dx   = self.target_world_x - self.px
        dy   = self.target_world_y - self.py
        dist = math.sqrt(dx * dx + dy * dy)

        self.get_logger().info(
            f'APPROACH  dist={dist:.3f} m to {self.target_pin}  z={self.pz:.2f} m',
            throttle_duration_sec=1.0)

        if dist < 1.0:
            self._stop()
            self.hover_start = time.time()
            self._go(HOVER)
            return

        target_yaw = math.atan2(dy, dx)
        yaw_err = target_yaw - self.yaw
        if yaw_err >  math.pi: yaw_err -= 2 * math.pi
        if yaw_err < -math.pi: yaw_err += 2 * math.pi

        align = max(0.0, math.cos(yaw_err))  # don't charge forward mid-turn
        t = Twist()
        t.linear.x  = min(0.5, dist * 0.3) * align
        t.angular.z = yaw_err * 1.5
        alt_err     = self.cruise_alt - self.pz
        t.linear.z  = max(-0.5, min(0.5, alt_err * 1.5))
        self.cmd_pub.publish(t)

    # ── HOVER: hold position over target, print AEROPIN, then land ───

    def _do_hover(self):
        # Altitude hold while hovering
        t = Twist()
        alt_err = self.cruise_alt - self.pz
        t.linear.z = max(-0.5, min(0.5, alt_err * 1.5))
        self.cmd_pub.publish(t)

        elapsed = time.time() - self.hover_start

        if elapsed < 1.0:
            return

        if elapsed < 1.5:
            # Print the result once
            self.get_logger().info('')
            self.get_logger().info('╔══════════════════════════════════════════════╗')
            self.get_logger().info(f'║  🎯 TARGET: "{self.target_class}"                       ║')
            self.get_logger().info(f'║  📍 AEROPIN: {self.target_pin}                       ║')
            self.get_logger().info(f'║  🌍 Position: ({self.target_world_x:.2f}, {self.target_world_y:.2f}) m          ║')
            self.get_logger().info(f'║  🚁 Drone at: ({self.px:.2f}, {self.py:.2f}, {self.pz:.2f}) m     ║')
            self.get_logger().info('╚══════════════════════════════════════════════╝')
            self.get_logger().info('')
            return

        if elapsed > 5.0:
            self.get_logger().info('Landing...')
            self._stop()
            self.land_pub.publish(Empty())
            self.land_t = time.time()
            self._go(LAND)

    # ── LAND ─────────────────────────────────────────────────────────

    def _do_land(self):
        self._stop()
        if self.land_t and time.time() - self.land_t >= 5.0:
            self.get_logger().info('═══ SEARCH MISSION COMPLETE ═══')
            if self.found:
                self.get_logger().info(
                    f'Result: "{self.target_class}" found at '
                    f'AEROPIN {self.target_pin}  '
                    f'({self.target_world_x:.2f}, {self.target_world_y:.2f}) m')
            else:
                self.get_logger().info(
                    f'Result: "{self.target_class}" not found in search area.')
            self._go(IDLE)

    # ── Helpers ──────────────────────────────────────────────────────

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _go(self, s: str):
        self.state = s
        self.get_logger().info(f'──── {s.upper()} ────')


def main(args=None):
    _ensure_yolo()

    target = _prompt_target()
    print(f'  Starting search for "{target}" ...')
    print()

    rclpy.init(args=args)
    node = ObjectSearchController(target)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()