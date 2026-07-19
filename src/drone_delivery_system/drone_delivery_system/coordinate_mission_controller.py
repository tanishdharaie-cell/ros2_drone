#!/usr/bin/env python3
"""
coordinate_mission_controller.py

Flies to a decoded AeroPin target, lands, drops payload, then reuses the
same proven state machine to fly home and land for good. No re-takeoff
retry/leveling machinery -- takeoff is only ever called once per landing.
"""

import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import Empty, String, Int8, Bool
from ros_gz_interfaces.msg import Contacts
from std_srvs.srv import Empty as EmptySrv

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
from aeropin import decode as aeropin_decode, validate as aeropin_validate
from aeropin import CELL_SIZE_M

LANDED_MODEL    = 0
TAKINGOFF_MODEL = 1
FLYING_MODEL    = 2
LANDING_MODEL   = 3

IDLE      = 'idle'
TAKEOFF   = 'takeoff'
CLIMB     = 'climb'
GOTO      = 'goto'
DESCEND   = 'descend'
LAND      = 'land'

KNOWN = {
    'K22-222-22': 'Origin           ( 0.00,  0.00) m',
    'K2J-F64-3M': 'Person Standing  ( 4.40,  2.40) m',
    'K2K-97F-PM': 'Dumpster         ( 3.71,  4.45) m',
    'J54-95C-6C': 'Fire Hydrant     ( 0.45, -1.66) m',
    'J57-K47-PC': 'Cardboard Boxes  ( 2.39, -3.68) m',
    '8CT-PP5-CM': 'Table            (-6.33,  5.25) m',
    'K22-772-7T': 'Test point       ( 0.50,  0.50) m',
    'K22-222-7K': 'Test point       ( 0.01,  0.01) m',
    'K2J-M93-5P': 'Near Person S    ( 4.40,  1.90) m  [0.5m south]',
    'K2J-T74-5M': 'Near Person N    ( 4.40,  2.90) m  [0.5m north]',
    'K2P-4C4-JM': 'Near Person E    ( 4.90,  2.40) m  [0.5m east]',
    'K2J-8MF-JM': 'Near Person W    ( 3.90,  2.40) m  [0.5m west]',
}


def _prompt() -> tuple:
    print()
    print('  ROS2 Drone -- AEROPIN Mission Controller')
    print()
    for code, label in KNOWN.items():
        print(f'    {code}  ->  {label}')
    print()
    while True:
        try:
            raw = input('  Enter AEROPIN > ').strip()
            if not raw:
                continue
            ok, err = aeropin_validate(raw)
            if not ok:
                print(f'  X  {err}')
                continue
            x, y = aeropin_decode(raw)
            print(f'  OK  ({x:.4f}, {y:.4f}) m')
            return x, y
        except (EOFError, KeyboardInterrupt):
            print('\n  Aborted.')
            sys.exit(0)


class CoordinateMissionController(Node):

    def __init__(self, target_x: float, target_y: float):
        super().__init__('coordinate_mission_controller')

        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])

        self.declare_parameter('drone_namespace', 'simple_drone')
        self.declare_parameter('cruise_alt',  3.0)
        self.declare_parameter('stop_radius', 0.25)
        self.declare_parameter('land_sonar',  0.5)
        self.declare_parameter('sonar_topic', '/simple_drone/sonar')
        self.declare_parameter('odom_topic',  '/simple_drone/odom')

        ns               = self.get_parameter('drone_namespace').value
        self.cruise_alt  = self.get_parameter('cruise_alt').value
        self.stop_radius = self.get_parameter('stop_radius').value
        self.land_sonar  = self.get_parameter('land_sonar').value

        self.target_x = target_x
        self.target_y = target_y

        self.state       = IDLE
        self.px = self.py = self.pz = 0.0
        self.yaw = self.roll = self.pitch = 0.0
        self.sonar       = 99.0
        self.land_t      = None
        self.carrying    = True
        self.vz              = 0.0
        self.drone_state = LANDED_MODEL
        self.collision_detected = False
        self.retreat_until = None
        self.origin_x = None
        self.origin_y = None
        self.takeoff_sent_t = None
        self.homebound = False
        self._drop_timer = None

        self.cmd_pub     = self.create_publisher(Twist,  f'/{ns}/cmd_vel',  10)
        self.takeoff_pub = self.create_publisher(Empty,  f'/{ns}/takeoff',  10)
        self.land_pub    = self.create_publisher(Empty,  f'/{ns}/land',     10)
        self.posctrl_pub = self.create_publisher(Bool,   f'/{ns}/posctrl',  10)
        self.state_pub   = self.create_publisher(String, '/delivery/state', 10)
        self.drop_cli    = self.create_client(EmptySrv, '/payload/drop')

        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._odom_cb, 10)
        self.create_subscription(
            Range, self.get_parameter('sonar_topic').value, self._sonar_cb, 10)
        self.create_subscription(
            Contacts, f'/{ns}/collision', self._collision_cb, 10)
        self.create_subscription(
            Int8, f'/{ns}/state', self._drone_state_cb, 10)

        self.create_service(EmptySrv, '/delivery/start', self._start_cb)
        self.create_timer(0.1, self._loop)

        self.get_logger().info(
            f'Target ({self.target_x:.4f}, {self.target_y:.4f}) m  |  '
            f'cruise={self.cruise_alt}m  stop={self.stop_radius}m  '
            f'land_sonar={self.land_sonar}m')
        self.get_logger().info('Auto-starting in 2 s ...')
        self._start_timer = self.create_timer(2.0, self._auto_start)

    def _auto_start(self):
        if self.state != IDLE:
            return
        self._start_timer.cancel()
        self.get_logger().info('=== MISSION START -- sending takeoff ===')
        b = Bool(); b.data = False
        self.posctrl_pub.publish(b)
        self.takeoff_pub.publish(Empty())
        self.takeoff_sent_t = time.time()
        self._go(TAKEOFF)

    def _start_cb(self, _req, res):
        self._auto_start()
        return res

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.px, self.py, self.pz = p.x, p.y, p.z
        self.vz = msg.twist.twist.linear.z
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
        self.roll = math.atan2(
            2.0 * (q.w * q.x + q.y * q.z),
            1.0 - 2.0 * (q.x ** 2 + q.y ** 2))
        self.pitch = math.asin(
            max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))

    def _sonar_cb(self, msg: Range):
        self.sonar = msg.range

    def _collision_cb(self, msg: Contacts):
        if len(msg.contacts) > 0:
            if not self.collision_detected:
                self.get_logger().warn('Collision detected -- retreating')
            self.collision_detected = True
            self.retreat_until = time.time() + 1.5

    def _drone_state_cb(self, msg: Int8):
        self.drone_state = msg.data

    def _loop(self):
        SAFETY_CEILING = self.cruise_alt + 2.0
        if self.pz > SAFETY_CEILING and self.state not in (IDLE, LAND):
            self.get_logger().error(
                f'SAFETY: altitude {self.pz:.2f}m exceeds ceiling '
                f'{SAFETY_CEILING:.2f}m -- forcing descent',
                throttle_duration_sec=1.0)
            t = Twist()
            t.linear.z = -1.0
            self.cmd_pub.publish(t)
            return
        {TAKEOFF: self._do_takeoff,
         CLIMB:   self._do_climb,
         GOTO:    self._do_goto,
         DESCEND: self._do_descend,
         LAND:    self._do_land,
         }.get(self.state, lambda: None)()

    def _do_takeoff(self):
        self.get_logger().info(
            f'TAKEOFF  waiting for FLYING state '
            f'(drone_state={self.drone_state})',
            throttle_duration_sec=1.0)

        if self.drone_state == FLYING_MODEL:
            if not self.homebound:
                self.origin_x = self.px
                self.origin_y = self.py
            self.get_logger().info('Drone is FLYING -> CLIMB')
            self._go(CLIMB)
            return

        if self.drone_state == LANDED_MODEL:
            if self.takeoff_sent_t and time.time() - self.takeoff_sent_t > 5.0:
                self.get_logger().warn('Resending takeoff ...')
                self.takeoff_pub.publish(Empty())
                self.takeoff_sent_t = time.time()

    def _do_climb(self):
        self.get_logger().info(
            f'CLIMB  z={self.pz:.2f}m -> {self.cruise_alt:.2f}m',
            throttle_duration_sec=1.0)
        err = self.cruise_alt - self.pz
        if abs(err) < 0.05:
            self._stop()
            self._go(GOTO)
            return
        speed = max(0.3, min(0.5, abs(err)))
        t = Twist()
        t.linear.z = speed if err > 0 else -speed
        self.cmd_pub.publish(t)

    def _do_goto(self):
        if self.collision_detected:
            if self.retreat_until and time.time() < self.retreat_until:
                t = Twist()
                t.linear.x = -0.4
                self.cmd_pub.publish(t)
                return
            else:
                self.collision_detected = False
                self.retreat_until = None

        tx, ty = (self.origin_x, self.origin_y) if self.homebound \
            else (self.target_x, self.target_y)

        dx = tx - self.px
        dy = ty - self.py
        dist = math.hypot(dx, dy)

        self.get_logger().info(
            f'GOTO {"(home)" if self.homebound else "(target)"}  '
            f'pos=({self.px:.2f},{self.py:.2f})  dist={dist:.2f}m',
            throttle_duration_sec=1.0)

        if dist <= self.stop_radius:
            self._stop()
            self._go(DESCEND)
            return

        heading = math.atan2(dy, dx)
        yaw_err = heading - self.yaw
        yaw_err = (yaw_err + math.pi) % (2 * math.pi) - math.pi
        align = max(0.0, math.cos(yaw_err))

        t = Twist()
        t.linear.x = max(0.2, min(1.0, dist - self.stop_radius)) * align
        t.angular.z = max(-1.0, min(1.0, yaw_err))
        t.linear.z = max(-0.5, min(0.5, self.cruise_alt - self.pz))
        self.cmd_pub.publish(t)

    def _do_descend(self):
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

        self.get_logger().info(
            f'DESCEND  z={self.pz:.3f} m  vz={self.vz:+.3f} m/s  sonar={self.sonar:.3f} m',
            throttle_duration_sec=0.5)

        if self.sonar <= self.land_sonar:
            self.get_logger().info(f'sonar={self.sonar:.3f} m -> LAND')
            self._stop(); self._go(LAND); return

        if self.vz < -0.5:
            t = Twist(); t.linear.z = 1.0
            self.cmd_pub.publish(t)
        else:
            t = Twist(); t.linear.z = -2.0
            self.cmd_pub.publish(t)

    def _do_land(self):
        self._stop()
        if self.land_t is None:
            self.get_logger().info(
                f'LAND {"(final -- at home)" if self.homebound else "(at target)"}')
            self.land_pub.publish(Empty())
            self.land_t = time.time()
            if not self.homebound:
                if self._drop_timer is not None:
                    self._drop_timer.cancel()
                self._drop_timer = self.create_timer(2.0, self._auto_drop)
            return

        if time.time() - self.land_t < 7.0:
            return

        if self.homebound:
            self.get_logger().info('=== MISSION COMPLETE -- home and landed ===')
            self._go(IDLE)
            return

        self.get_logger().info('=== Delivery complete -- heading home ===')
        self.homebound = True
        self.land_t = None
        b = Bool(); b.data = False
        self.posctrl_pub.publish(b)
        self.takeoff_pub.publish(Empty())
        self.takeoff_sent_t = time.time()
        self._go(TAKEOFF)

    def _auto_drop(self):
        if self._drop_timer is not None:
            self._drop_timer.cancel()
            self._drop_timer = None
        if not self.carrying:
            return
        if self.drop_cli.service_is_ready():
            self.drop_cli.call_async(EmptySrv.Request())
            self.carrying = False
        else:
            self.get_logger().warn(
                "payload_manager not running -- cannot drop payload")

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _go(self, s: str):
        self.state = s
        m = String(); m.data = s
        self.state_pub.publish(m)
        self.get_logger().info(f'---- {s.upper()} ----')


def main(args=None):
    tx, ty = _prompt()
    print(f'  Starting ROS2 node -- target=({tx:.4f}, {ty:.4f}) m')
    rclpy.init(args=args)
    node = CoordinateMissionController(tx, ty)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
