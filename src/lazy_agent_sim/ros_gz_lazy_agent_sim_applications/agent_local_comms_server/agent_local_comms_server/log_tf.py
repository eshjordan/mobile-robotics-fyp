#!/usr/bin/env python3

import rclpy
import rclpy.clock_type
import rclpy.time

import transforms3d as tf
import numpy as np

from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
import rosbag2_py
from rosbag2_interfaces.srv import (
    Play as PlayRosbag,
    Stop as StopRosbag,
    Resume as ResumeRosbag,
    Pause as PauseRosbag,
    IsPaused as IsPausedRosbag,
)



frames = [
    'epuck2_robot_5785',
    'epuck2_robot_5653',
    'epuck2_robot_5731',
    'epuck2_robot_5831',
]


class TfLogger(Node):
    def __init__(self):
        super().__init__('tf_logger')
        self.get_logger().info('Starting tf logger')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_mocap_buffer = Buffer()
        self.tf_mocap_listener = TransformListener(self.tf_mocap_buffer, self)

        tf_sub_msg_type = self.tf_mocap_listener.tf_sub.msg_type
        tf_sub_callback = self.tf_mocap_listener.tf_sub.callback
        tf_sub_qos_profile = self.tf_mocap_listener.tf_sub.qos_profile
        tf_sub_callback_group = self.tf_mocap_listener.tf_sub.callback_group

        self.tf_mocap_listener.node.destroy_subscription(self.tf_mocap_listener.tf_sub)
        del self.tf_mocap_listener.tf_sub

        new_tf_sub = self.tf_mocap_listener.node.create_subscription(
            tf_sub_msg_type,
            '/tf_mocap',
            tf_sub_callback,
            tf_sub_qos_profile,
            callback_group=tf_sub_callback_group
        )

        new_tf_static_sub = self.tf_mocap_listener.node.create_subscription(
            self.tf_mocap_listener.tf_static_sub.msg_type,
            '/tf_static_mocap',
            self.tf_mocap_listener.tf_static_sub.callback,
            self.tf_mocap_listener.tf_static_sub.qos_profile,
            callback_group=self.tf_mocap_listener.tf_static_sub.callback_group
        )

        self.tf_mocap_listener.node.destroy_subscription(self.tf_mocap_listener.tf_static_sub)
        del self.tf_mocap_listener.tf_static_sub

        self.tf_mocap_listener.tf_sub = new_tf_sub
        self.tf_mocap_listener.tf_static_sub = new_tf_static_sub

        self.rosbag_client_play = self.create_client(PlayRosbag, '/rosbag_player/play')
        self.rosbag_client_stop = self.create_client(StopRosbag, '/rosbag_player/stop')
        self.rosbag_client_resume = self.create_client(ResumeRosbag, '/rosbag_player/resume')
        self.rosbag_client_pause = self.create_client(PauseRosbag, '/rosbag_player/pause')
        self.rosbag_client_is_paused = self.create_client(IsPausedRosbag, '/rosbag_player/is_paused')

        def resume_rosbag():
            def handle_future_issue(future: rclpy.Future):
                if future.cancelled() or future.exception():
                    self.get_logger().error('Service call to rosbag player failed')
                    self.shutdown()
                    return False

            def handle_resume_response(future: rclpy.Future):
                if self.log_tf_tmr.is_canceled():
                    self.log_tf_tmr.reset()

            def handle_is_paused_response(future: rclpy.Future):
                if future.result().paused:
                    response = self.rosbag_client_resume.call_async(ResumeRosbag.Request())
                    response.add_done_callback(handle_future_issue)
                    response.add_done_callback(handle_resume_response)

            is_paused_future = self.rosbag_client_is_paused.call_async(IsPausedRosbag.Request())
            is_paused_future.add_done_callback(handle_future_issue)
            is_paused_future.add_done_callback(handle_is_paused_response)

            self._resume_tmr.cancel()

        self._resume_tmr = self.create_timer(2.0, resume_rosbag)
        self.log_tf_tmr = self.create_timer(0.1, self.log_tf, autostart=False)

    def shutdown(self):
        self.get_logger().warning('Shutting down tf logger')
        self.tf_listener.unregister()
        self.tf_mocap_listener.unregister()
        self.destroy_node()

    def log_tf(self):
        # out = self.tf_buffer.all_frames_as_yaml()
        # self.get_logger().info(out)
        for frame in frames:
            if not self.tf_buffer.can_transform('earth', frame, rclpy.time.Time(clock_type=rclpy.clock_type.ClockType.ROS_TIME)):
                self.get_logger().debug("Can't transform tf")
                continue

            tf_sim = self.tf_buffer.lookup_transform('earth', frame, rclpy.time.Time(clock_type=rclpy.clock_type.ClockType.ROS_TIME))
            sim_x = tf_sim.transform.translation.x
            sim_y = tf_sim.transform.translation.y
            sim_z = tf_sim.transform.translation.z
            sim_qx = tf_sim.transform.rotation.x
            sim_qy = tf_sim.transform.rotation.y
            sim_qz = tf_sim.transform.rotation.z
            sim_qw = tf_sim.transform.rotation.w

            if not self.tf_mocap_buffer.can_transform('earth', frame, rclpy.time.Time(clock_type=rclpy.clock_type.ClockType.ROS_TIME)):
                self.get_logger().debug("Can't transform tf_mocap")
                continue

            tf_bag = self.tf_mocap_buffer.lookup_transform('earth', frame, rclpy.time.Time(clock_type=rclpy.clock_type.ClockType.ROS_TIME))
            bag_x = tf_bag.transform.translation.x
            bag_y = tf_bag.transform.translation.y
            bag_z = tf_bag.transform.translation.z
            bag_qx = tf_bag.transform.rotation.x
            bag_qy = tf_bag.transform.rotation.y
            bag_qz = tf_bag.transform.rotation.z
            bag_qw = tf_bag.transform.rotation.w

            diff = np.array([sim_x-bag_x, sim_y-bag_y, sim_z-bag_z])
            dist = np.linalg.norm(diff)

            self.get_logger().info(f"{frame}: sim-bag: {dist} ({diff})")



def main():
    rclpy.init()
    node = TfLogger()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
