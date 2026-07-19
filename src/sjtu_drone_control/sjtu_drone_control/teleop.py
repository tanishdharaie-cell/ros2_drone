#!/usr/bin/env python3
# Copyright 2023 Georg Novotny
#
# Licensed under the GNU GENERAL PUBLIC LICENSE, Version 3.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.gnu.org/licenses/gpl-3.0.en.html
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Empty
import sys
import termios
import tty


MSG = """
Control Your Drone!
---------------------------
Moving around:
        w
    a   s    d
        x

t/l: takeoff/land (upper/lower case)
q/e : increase/decrease linear and angular velocity (upper/lower case)
A/D: rotate left/right
r/f : rise/fall (upper/lower case)

---------------------------
CTRL-C to quit
---------------------------

"""


class TeleopNode(Node):
    def __init__(self) -> None:
        super().__init__('teleop_node')

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.takeoff_publisher = self.create_publisher(Empty, 'takeoff', 10)
        self.land_publisher = self.create_publisher(Empty, 'land', 10)

        # Velocity parameters
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.linear_increment = 0.05
        self.angular_increment = 0.05
        self.max_linear_velocity = 1.0
        self.max_angular_velocity = 1.0

        # Start a timer to listen to keyboard inputs
        self.create_timer((1/30), self.read_keyboard_input)

    def get_velocity_msg(self) -> str:
        return "Linear Velocity: " + str(self.linear_velocity) + "\nAngular Velocity: " \
            + str(self.angular_velocity) + "\n"

    def read_keyboard_input(self) -> None:
        """
        Read keyboard inputs and publish corresponding commands
        """
        while rclpy.ok():
            # Print the instructions
            print(MSG+self.get_velocity_msg())
            # Implement a non-blocking keyboard read
            key = self.get_key()

            # ==========================================================
            # TODO 1
            #
            # Handle keyboard input and publish the corresponding
            # drone command.
            #
            # Requirements:
            # - q/e (case-insensitive): increase/decrease both
            #   linear_velocity and angular_velocity by their
            #   respective increments, clamped to
            #   [-max_*_velocity, +max_*_velocity].
            # - w/x (case-insensitive): publish forward/backward
            #   motion using linear_velocity on the X axis.
            # - s (case-insensitive): publish a zero-velocity
            #   Twist (hover).
            # - a/d: publish left/right motion using
            #   linear_velocity on the Y axis.
            # - A/D (uppercase): publish left/right yaw rotation
            #   using angular_velocity on the Z axis.
            # - r/f (case-insensitive): publish rise/fall motion
            #   using velocity on the Z axis.
            # - t (case-insensitive): publish a takeoff command.
            # - l (case-insensitive): publish a zero-velocity
            #   Twist, then publish a land command.
            #
            # Hint:
            # Use:
            #   • self.linear_velocity, self.angular_velocity
            #   • self.linear_increment, self.angular_increment
            #   • self.max_linear_velocity, self.max_angular_velocity
            #   • Vector3(), self.publish_cmd_vel()
            #   • self.takeoff_publisher, self.land_publisher
            # ==========================================================

            # YOUR CODE HERE
            #q/e — velocity increment/decrement, clamped:
            if key.lower() == 'q':
                self.linear_velocity = max(-self.max_linear_velocity,
                                min(self.max_linear_velocity,
                                    self.linear_velocity + self.linear_increment))
                self.angular_velocity = max(-self.max_angular_velocity,
                                 min(self.max_angular_velocity,
                                     self.angular_velocity + self.angular_increment))

            elif key.lower() == 'e':
                self.linear_velocity = max(-self.max_linear_velocity,
                                min(self.max_linear_velocity,
                                    self.linear_velocity - self.linear_increment))
                self.angular_velocity = max(-self.max_angular_velocity,
                                 min(self.max_angular_velocity,
                                     self.angular_velocity - self.angular_increment))
    
            #w/x — forward/backward:
            elif key.lower() == 'w':
                self.publish_cmd_vel(linear_vec=Vector3(x=self.linear_velocity))
            elif key.lower() == 'x':
                self.publish_cmd_vel(linear_vec=Vector3(x=-self.linear_velocity))
            #s — hover (zero Twist):
            elif key.lower() == 's':
                self.publish_cmd_vel()
            #a/d — exact case, strafe on Y: same pattern as w/x but Vector3(y=...), and checked as key == 'a' / key == 'd'
            elif key == 'a':
                self.publish_cmd_vel(linear_vec=Vector3(y=self.linear_velocity))
            elif key == 'd':
                self.publish_cmd_vel(linear_vec=Vector3(y=-self.linear_velocity))
            #A/D — exact case, yaw on Z, uses angular_vec= not linear_vec=:
            elif key == 'A':
                self.publish_cmd_vel(angular_vec=Vector3(z=self.angular_velocity))
            elif key == 'D':
                self.publish_cmd_vel(angular_vec=Vector3(z=-self.angular_velocity))
            #r/f — rise/fall on Z: same pattern as w/x, Vector3(z=...)
            elif key.lower() == 'r':
                self.publish_cmd_vel(linear_vec=Vector3(z=self.linear_velocity))
            elif key.lower() == 'f':
                self.publish_cmd_vel(linear_vec=Vector3(z=-self.linear_velocity))
            #t — takeoff:
            elif key.lower() == 't':
                self.takeoff_publisher.publish(Empty())
            #l — stop, then land:
            elif key.lower() == 'l':
                self.publish_cmd_vel()
                self.land_publisher.publish(Empty())
            

            

    def get_key(self) -> str:
        """
        Function to capture keyboard input
        """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def publish_cmd_vel(self, linear_vec: Vector3 = Vector3(),
                        angular_vec: Vector3 = Vector3()) -> None:
        """
        Publish a Twist message to cmd_vel topic
        """
        twist = Twist(linear=linear_vec, angular=angular_vec)
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopNode()
    rclpy.spin(teleop_node)
    teleop_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()