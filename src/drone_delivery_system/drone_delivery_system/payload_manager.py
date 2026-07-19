#!/usr/bin/env python3
"""
Payload Manager
On /payload/drop: spawns a free-falling red cube at drone position
via Gazebo EntityFactory, simulating the drop.
"""

import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty as EmptySrv
from nav_msgs.msg import Odometry
from ros_gz_interfaces.srv import SpawnEntity


_DROPPED_CUBE_SDF = """<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="dropped_cube">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.0001667</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.0001667</iyy><iyz>0</iyz>
          <izz>0.0001667</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>0.1 0.1 0.1</size></box></geometry>
        <surface>
          <friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction>
          <bounce><restitution_coefficient>0</restitution_coefficient></bounce>
        </surface>
      </collision>
      <visual name="visual">
        <geometry><box><size>0.1 0.1 0.1</size></box></geometry>
        <material>
          <ambient>1.0 0.0 0.0 1.0</ambient>
          <diffuse>1.0 0.0 0.0 1.0</diffuse>
          <specular>0.5 0.5 0.5 1.0</specular>
          <emissive>0.2 0.0 0.0 1.0</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


class PayloadManager(Node):
    def __init__(self):
        super().__init__('payload_manager')

        self.declare_parameter('odom_topic',    '/simple_drone/odom')
        self.declare_parameter('world_name',    'empty')
        self.declare_parameter('cube_offset_z', -0.15)

        odom_topic      = self.get_parameter('odom_topic').value
        self.world_name = self.get_parameter('world_name').value
        self.offset_z   = self.get_parameter('cube_offset_z').value

        self.carrying = True
        self.drone_x = self.drone_y = self.drone_z = 0.0

        self.status_pub = self.create_publisher(String, '/payload/status',   10)
        self.bool_pub   = self.create_publisher(Bool,   '/payload/attached', 10)

        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.create_service(EmptySrv, '/payload/drop', self._drop_cb)

        spawn_srv = f'/world/{self.world_name}/create'
        self.spawn_cli = self.create_client(SpawnEntity, spawn_srv)

        self.get_logger().info(f'PayloadManager ready  spawn={spawn_srv}')
        self._pub_status(True)

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.drone_x, self.drone_y, self.drone_z = p.x, p.y, p.z

    def _drop_cb(self, _req, res):
        if not self.carrying:
            self.get_logger().warn("Already dropped!")
            return res
        cx = self.drone_x
        cy = self.drone_y
        cz = self.drone_z + self.offset_z
        self.get_logger().info(f"📦 Dropping at ({cx:.2f}, {cy:.2f}, {cz:.2f})")
        sdf_escaped = _DROPPED_CUBE_SDF.replace(chr(34), chr(92) + chr(34)).replace(chr(10), " ")
        gz_req = f"sdf: \"{sdf_escaped}\", name: \"dropped_cube\", pose: {{position: {{x: {cx}, y: {cy}, z: {cz}}}}}"
        result = subprocess.run(
            ["gz", "service", "-s", f"/world/{self.world_name}/create",
             "--reqtype", "gz.msgs.EntityFactory",
             "--reptype", "gz.msgs.Boolean",
             "--timeout", "3000",
             "--req", gz_req],
            capture_output=True, text=True,
        )
        if "data: true" in result.stdout:
            self.get_logger().info("✓ Payload spawned successfully")
            self.carrying = False
            self._pub_status(False)
        else:
            self.get_logger().warn(f"Spawn failed: {result.stdout} {result.stderr}")
        return res

    def _pub_status(self, carrying: bool):
        s = String(); s.data = 'carrying' if carrying else 'dropped'
        self.status_pub.publish(s)
        b = Bool(); b.data = carrying
        self.bool_pub.publish(b)


def main(args=None):
    rclpy.init(args=args)
    node = PayloadManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()