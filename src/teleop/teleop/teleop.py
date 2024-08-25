# Copyright (C) 2024  Jordan Esh.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import rclpy.node
import geometry_msgs.msg
from pynput.keyboard import Key, Listener


class Teleop(rclpy.node.Node):
    def __init__(self):
        super().__init__("teleop")
        self.cmd_pub = self.create_publisher(
            geometry_msgs.msg.Twist,
            "mobile_base/cmd_vel",
            10,
        )
        self.ctrl_timer = self.create_timer(0.1, self.ctrl_callback)

        self.forward = False
        self.backward = False
        self.left = False
        self.right = False
        self.shift = False
        self.keyboard_listener = Listener(
            on_press=self.on_press, on_release=self.on_release
        )
        self.keyboard_listener.start()

    def ctrl_callback(self):
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        linear = (float(self.forward) - float(self.backward)) * 0.06
        angular = (float(self.left) - float(self.right)) * 1.75

        if linear != 0 and angular == 0:
            twist.linear.x = linear
        elif linear == 0 and angular != 0:
            twist.angular.z = angular
        elif linear != 0 and angular != 0:
            twist.linear.x = linear
            twist.angular.z = angular / 15.0

        if self.shift:
            twist.linear.x *= 2.0
            twist.angular.z *= 2.0

        self.cmd_pub.publish(twist)

    def on_press(self, key):
        if key == Key.up:
            self.forward = True
        elif key == Key.down:
            self.backward = True
        elif key == Key.left:
            self.left = True
        elif key == Key.right:
            self.right = True
        elif key == Key.shift:
            self.shift = True

    def on_release(self, key):
        if key == Key.up:
            self.forward = False
        elif key == Key.down:
            self.backward = False
        elif key == Key.left:
            self.left = False
        elif key == Key.right:
            self.right = False
        elif key == Key.shift:
            self.shift = False


def main():
    rclpy.init()
    teleop = Teleop()
    rclpy.spin(teleop)


if __name__ == "__main__":
    main()
