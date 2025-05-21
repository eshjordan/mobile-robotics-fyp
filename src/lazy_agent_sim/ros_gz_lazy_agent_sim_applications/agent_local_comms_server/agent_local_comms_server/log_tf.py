#!/usr/bin/env python3

import rclpy
import rclpy.time

from rclpy.node import Node
from tf2_ros import TransformListener, Buffer

frames = [
    'epuck2_robot_5785',
    'epuck2_robot_5653',
    'epuck2_robot_5731',
    'epuck2_robot_5831',
]


class TfLogger(Node):
    def __init__(self):
        super().__init__('tf_logger')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self.log_tf)

    def log_tf(self):
        out = self.tf_buffer.all_frames_as_yaml()
        self.get_logger().info(out)
        for frame in frames:
            tf = self.tf_buffer.lookup_transform('earth', frame, rclpy.time.Time())
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            z = tf.transform.translation.z
            qx = tf.transform.rotation.x
            qy = tf.transform.rotation.y
            qz = tf.transform.rotation.z
            qw = tf.transform.rotation.w
            


def main():
    rclpy.init()
    node = TfLogger()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
