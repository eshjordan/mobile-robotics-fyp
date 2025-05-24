#!/usr/bin/env python3

import asyncio
import asyncio.coroutines
import asyncio.taskgroups
import asyncio.tasks
import functools
import itertools
from typing import Optional, Union
import geometry_msgs.msg
import rclpy
import rclpy.clock_type
import rclpy.task
import rclpy.time


import numpy as np
import time

from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import ReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from threading import Thread
import rosbag2_py
from rosbag2_interfaces.srv import (
    Play as PlayRosbag,
    Stop as StopRosbag,
    Resume as ResumeRosbag,
    Pause as PauseRosbag,
    IsPaused as IsPausedRosbag,
)
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.backend_bases

from csv import DictWriter

import transforms3d as tf
def quat_string_to_euler(val):
    return " ".join(map(str, tf.euler.quat2euler([float(v[1]) for v in sorted(dict([tuple(a.split('=')) for a in [s.strip() for s in val.split(",")]]).items())])))

gz_topics = """
/epuck2_robot_5831/camera
/epuck2_robot_5831/dist_sens
/epuck2_robot_5831/imu
/epuck2_robot_5831/joint_states
/epuck2_robot_5831/mobile_base/cmd_vel
/epuck2_robot_5831/odom
/epuck2_robot_5831/pose
/epuck2_robot_5831/proximity0
/epuck2_robot_5831/proximity1
/epuck2_robot_5831/proximity2
/epuck2_robot_5831/proximity3
/epuck2_robot_5831/proximity4
/epuck2_robot_5831/proximity5
/epuck2_robot_5831/proximity6
/epuck2_robot_5831/proximity7
/epuck2_robot_5831/robot_description
/epuck2_robot_5831/scan
"""


bag_topics = """
/epuck2_robot_5653/battery
/epuck2_robot_5653/dist_sens
/epuck2_robot_5653/imu
/epuck2_robot_5653/joint_states
/epuck2_robot_5653/mag_field
/epuck2_robot_5653/mag_field_vector
/epuck2_robot_5653/microphone
/epuck2_robot_5653/mobile_base/cmd_vel
/epuck2_robot_5653/odom
/epuck2_robot_5653/proximity0
/epuck2_robot_5653/proximity1
/epuck2_robot_5653/proximity2
/epuck2_robot_5653/proximity3
/epuck2_robot_5653/proximity4
/epuck2_robot_5653/proximity5
/epuck2_robot_5653/proximity6
/epuck2_robot_5653/proximity7
/epuck2_robot_5653/robot_description
/epuck2_robot_5653/scan
"""

robots = [
    'epuck2_robot_5785',
    'epuck2_robot_5653',
    'epuck2_robot_5731',
    'epuck2_robot_5831',
]

robot_colors = {
    robot: color for robot, color in zip(
        robots,
        ['red', 'blue', 'green', 'orange'],
    )
}


class TfLogger(Node):
    def __init__(self, use_pose: bool = True):
        super().__init__('tf_logger')
        self.get_logger().info('Starting tf logger')

        self.use_pose = use_pose

        self.df = pd.DataFrame(columns=[
            'timestamp',
            'robot',
            'sim_x', 'sim_y', 'sim_z',
            'sim_ex', 'sim_ey', 'sim_ez',
            'bag_x', 'bag_y', 'bag_z',
            'bag_ex', 'bag_ey', 'bag_ez',
            'dist',
            'diff_x', 'diff_y', 'diff_z',
        ])

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_mocap_buffer = Buffer()
        if self.use_pose:
            self.tf_mocap_listeners = []
            qos = QoSProfile(
                # reliability=ReliabilityPolicy.BEST_EFFORT,
                # durability=DurabilityPolicy.VOLATILE,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=100,
            )
            # for i in range(len(robots)):
            #     robot = robots[i]
            #     topic = f'/vrpn_mocap/{robot}/pose'
            #     sub = self.create_subscription(
            #         geometry_msgs.msg.PoseStamped,
            #         f'{topic}',
            #         lambda msg: self.handle_pose_cb(msg, f'{robot}'),
            #         qos,
            #     )
            #     self.tf_mocap_listeners.append(sub)

            self.epuck2_robot_5785_sub = self.create_subscription(geometry_msgs.msg.PoseStamped, '/vrpn_mocap/epuck2_robot_5785/pose', self.handle_epuck2_robot_5785_cb, qos)
            self.epuck2_robot_5653_sub = self.create_subscription(geometry_msgs.msg.PoseStamped, '/vrpn_mocap/epuck2_robot_5653/pose', self.handle_epuck2_robot_5653_cb, qos)
            self.epuck2_robot_5731_sub = self.create_subscription(geometry_msgs.msg.PoseStamped, '/vrpn_mocap/epuck2_robot_5731/pose', self.handle_epuck2_robot_5731_cb, qos)
            self.epuck2_robot_5831_sub = self.create_subscription(geometry_msgs.msg.PoseStamped, '/vrpn_mocap/epuck2_robot_5831/pose', self.handle_epuck2_robot_5831_cb, qos)
            # qos = QoSProfile(
            #     depth=100,
            #     durability=DurabilityPolicy.VOLATILE,
            #     history=HistoryPolicy.KEEP_LAST,
            # ),
            # self.tf_mocap_listeners = []
            # self.tf_mocap_listeners += [self.create_subscription(geometry_msgs.msg.PoseStamped, '/vrpn_mocap/epuck2_robot_5785/pose', self.handle_pose_fn, qos)]
            # self.tf_mocap_listeners += [self.create_subscription(geometry_msgs.msg.PoseStamped, '/vrpn_mocap/epuck2_robot_5653/pose', self.handle_pose_fn, qos)]
            # self.tf_mocap_listeners += [self.create_subscription(geometry_msgs.msg.PoseStamped, '/vrpn_mocap/epuck2_robot_5731/pose', self.handle_pose_fn, qos)]
            # self.tf_mocap_listeners += [self.create_subscription(geometry_msgs.msg.PoseStamped, '/vrpn_mocap/epuck2_robot_5831/pose', self.handle_pose_fn, qos)]
        else:
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
        self._save_tmr = self.create_timer(5.0, self.save)
        self.running_tasks: list[rclpy.task.Task] = []
        def cleanup_tasks():
            self.running_tasks = [task for task in self.running_tasks if task.done() or task.cancelled() or task.exception()]

        self._cleanup_tasks = self.create_timer(0.5, cleanup_tasks)
        self.log_tf_tmr = self.create_timer(0.1, self.lookup_tfs, autostart=False)

        plt.ion()
        self.cid = None
        self.positions_visibility = {}
        self.positions_fig = plt.figure()
        self.positions_ax = self.positions_fig.gca()
        self.positions_ani = FuncAnimation(self.positions_fig, self.animate_positions, interval=100, save_count=5)
        self.rmse_fig = plt.figure()
        # self.rmse_ani = FuncAnimation(self.rmse_fig, self.animate_rmse, interval=1000, save_count=10)
        # self.rmse_ax = self.rmse_fig.gca()

        def show():
            self.positions_fig.canvas.draw()
            self.positions_fig.canvas.flush_events()
            self.rmse_fig.canvas.draw()
            self.rmse_fig.canvas.flush_events()

        self._plt_tmr = self.create_timer(0.01, show)

    def save(self):
        self.positions_fig.canvas.draw()
        self.positions_fig.canvas.flush_events()
        self.positions_fig.savefig('/home/jordan/colcon_ws/tf_log_positions.png')
        self.rmse_fig.canvas.draw()
        self.rmse_fig.canvas.flush_events()
        self.rmse_fig.savefig('/home/jordan/colcon_ws/tf_log_rmse.png')
        self.df.to_csv('/home/jordan/colcon_ws/tf_log.csv', index=False)

    def shutdown(self):
        self.get_logger().warning('Shutting down tf logger')
        self.save()
        plt.ioff()
        plt.close(self.positions_fig)
        plt.close(self.rmse_fig)

        self.destroy_node()

    def __del__(self):
        self.shutdown()

    def handle_epuck2_robot_5785_cb(self, msg: geometry_msgs.msg.PoseStamped):
        self.handle_pose_cb(msg, 'epuck2_robot_5785')

    def handle_epuck2_robot_5653_cb(self, msg: geometry_msgs.msg.PoseStamped):
        self.handle_pose_cb(msg, 'epuck2_robot_5653')

    def handle_epuck2_robot_5731_cb(self, msg: geometry_msgs.msg.PoseStamped):
        self.handle_pose_cb(msg, 'epuck2_robot_5731')

    def handle_epuck2_robot_5831_cb(self, msg: geometry_msgs.msg.PoseStamped):
        self.handle_pose_cb(msg, 'epuck2_robot_5831')

    def handle_pose_cb(self, msg: geometry_msgs.msg.PoseStamped, robot: str):
        self.get_logger().info(f'Received pose for {robot}: {msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z}')
        tf = geometry_msgs.msg.TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'earth'
        tf.child_frame_id = robot + '/base_link'
        tf.transform.translation.x = msg.pose.position.x
        tf.transform.translation.y = msg.pose.position.y
        tf.transform.translation.z = msg.pose.position.z
        tf.transform.rotation.w = msg.pose.orientation.w
        tf.transform.rotation.x = msg.pose.orientation.x
        tf.transform.rotation.y = msg.pose.orientation.y
        tf.transform.rotation.z = msg.pose.orientation.z
        self.tf_mocap_buffer.set_transform(tf, 'mocap_pose_cb')

    def log_transforms(self, tf_sim: geometry_msgs.msg.TransformStamped, tf_bag: geometry_msgs.msg.TransformStamped, robot: str, current_time: rclpy.time.Time):
        self.get_logger().info(f'Logging transforms for {robot} at {current_time.seconds_nanoseconds()}')
        self.get_logger().info(f'{robot} Sim TF: {tf_sim.transform.translation.x}, {tf_sim.transform.translation.y}, {tf_sim.transform.translation.z}, {tf_sim.transform.rotation.x}, {tf_sim.transform.rotation.y}, {tf_sim.transform.rotation.z}, {tf_sim.transform.rotation.w}')
        self.get_logger().info(f'{robot} Bag TF: {tf_bag.transform.translation.x}, {tf_bag.transform.translation.y}, {tf_bag.transform.translation.z}, {tf_bag.transform.rotation.x}, {tf_bag.transform.rotation.y}, {tf_bag.transform.rotation.z}, {tf_bag.transform.rotation.w}')
        sim_x = tf_sim.transform.translation.x
        sim_y = tf_sim.transform.translation.y
        sim_z = tf_sim.transform.translation.z
        sim_qx = tf_sim.transform.rotation.x
        sim_qy = tf_sim.transform.rotation.y
        sim_qz = tf_sim.transform.rotation.z
        sim_qw = tf_sim.transform.rotation.w
        sim_ex, sim_ey, sim_ez = tf.euler.quat2euler([sim_qw, sim_qx, sim_qy, sim_qz])

        bag_x = tf_bag.transform.translation.x
        bag_y = tf_bag.transform.translation.y
        bag_z = tf_bag.transform.translation.z
        bag_qx = tf_bag.transform.rotation.x
        bag_qy = tf_bag.transform.rotation.y
        bag_qz = tf_bag.transform.rotation.z
        bag_qw = tf_bag.transform.rotation.w
        bag_ex, bag_ey, bag_ez = tf.euler.quat2euler([bag_qw, bag_qx, bag_qy, bag_qz])

        secs, nsecs = current_time.seconds_nanoseconds()
        current_time_secs = secs + nsecs / 1e9

        diff = np.array([sim_x-bag_x, sim_y-bag_y, sim_z-bag_z])
        dist = np.linalg.norm(diff)

        # Write to CSV
        new_df_row = len(self.df)
        self.df.loc[new_df_row] = [None] * len(self.df.columns)

        # Set timestamp
        self.df.loc[new_df_row, 'timestamp'] = current_time_secs
        self.df.loc[new_df_row, 'robot'] = robot
        self.df.loc[new_df_row, ['sim_x', 'sim_y', 'sim_z']] = [sim_x, sim_y, sim_z]
        self.df.loc[new_df_row, ['sim_ex', 'sim_ey', 'sim_ez']] = [sim_ex, sim_ey, sim_ez]
        self.df.loc[new_df_row, ['bag_x', 'bag_y', 'bag_z']] = [bag_x, bag_y, bag_z]
        self.df.loc[new_df_row, ['bag_ex', 'bag_ey', 'bag_ez']] = [bag_ex, bag_ey, bag_ez]
        self.df.loc[new_df_row, 'dist'] = dist
        self.df.loc[new_df_row, ['diff_x', 'diff_y', 'diff_z']] = diff

    def lookup_tfs(self):
        current_time = self.get_clock().now()

        for i in range(len(robots)):
            robot = robots[i]
            # can_tf = self.tf_buffer.can_transform('earth', robot + '/base_link', current_time, rclpy.time.Duration(seconds=0.045))
            # can_tf_mocap = self.tf_mocap_buffer.can_transform('earth', robot + '/base_link', current_time, rclpy.time.Duration(seconds=0.045))

            # tf_corot = self.tf_buffer.lookup_transform_async('earth', robot + '/base_link', current_time)

            # if not can_tf or not can_tf_mocap:
            #     output = "Can't transform"
            #     output += ' tf' if not can_tf else ''
            #     output += ' or' if not (can_tf or can_tf_mocap) else ''
            #     output += ' tf_mocap' if not can_tf_mocap else ''
            #     output += f" - {robot}"
            #     self.get_logger().info(output)
            #     continue

            self.running_tasks.append(rclpy.get_global_executor().create_task(self.tf_buffer.lookup_transform_async, 'earth', f'{robot}/base_link', rclpy.time.Time()))
            tf_task = self.running_tasks[-1]
            self.running_tasks.append(rclpy.get_global_executor().create_task(self.tf_mocap_buffer.lookup_transform_async, 'earth', f'{robot}/base_link', rclpy.time.Time()))
            tf_mocap_task = self.running_tasks[-1]


            def call_log_transforms(tf_task: rclpy.task.Task, tf_mocap_task, robot_name: str, current_time: rclpy.time.Time):
                def inner(this_task: rclpy.task.Task):
                    if this_task.cancelled() or this_task.exception():
                        self.get_logger().error(f'Task failed: {this_task.exception()}')
                        rclpy.shutdown()
                        return

                    if this_task == tf_task:
                        return

                    if tf_task.executing():
                        asyncio.wait_for(tf_task, timeout=5.0)

                    tf_sim = tf_task.result()
                    tf_bag = tf_mocap_task.result()

                    if tf_sim is None:
                        self.get_logger().error(f'tf_sim for {robot_name} is None')

                    if tf_bag is None:
                        self.get_logger().error(f'tf_bag for {robot_name} is None')

                    if tf_sim is None or tf_bag is None:
                        return

                    self.log_transforms(tf_sim, tf_bag, robot, current_time)
                return inner

            tf_task.add_done_callback(call_log_transforms(tf_task, tf_mocap_task, f"{robot}", current_time))
            tf_mocap_task.add_done_callback(call_log_transforms(tf_task, tf_mocap_task, f"{robot}", current_time))



    def animate_positions(self, i):
        self.get_logger().debug('Animating positions start')
        t1 = time.time()
        self.positions_ax.clear()
        self.positions_ax.set_title('TF Positions')
        self.positions_ax.set_xlabel('X (m)')
        self.positions_ax.set_ylabel('Y (m)')
        self.positions_ax.grid()

        # Show +-5m range
        self.positions_ax.set_xlim(-5, 5)
        self.positions_ax.set_ylim(-5, 5)
        self.positions_ax.set_aspect('equal', adjustable='box')

        robot_lines = {}

        for robot in robots:
            if robot not in self.df['robot'].values:
                continue

            df_frame = self.df[self.df['robot'] == robot]

            if robot not in self.positions_visibility:
                self.positions_visibility[robot] = {'Mocap': True, 'Sim': True}

            robot_lines[robot] = {}

            # Show the history of the robot's position as a line
            # Show the latest position as a point
            mocap_alpha = 1.0 if self.positions_visibility[robot]['Mocap'] else 0.0
            sim_alpha = 0.5 if self.positions_visibility[robot]['Sim'] else 0.0

            self.positions_ax.plot(df_frame['bag_x'], df_frame['bag_y'], color=robot_colors[robot], linestyle='-', label=f'Mocap {robot}', alpha=mocap_alpha)
            self.positions_ax.scatter(df_frame['bag_x'].iloc[-1], df_frame['bag_y'].iloc[-1], color=robot_colors[robot], marker='o', s=100, alpha=mocap_alpha, label='_Latest Bag Position')

            self.positions_ax.plot(df_frame['sim_x'], df_frame['sim_y'], color='black', linestyle='--', label=f'Sim {robot}', alpha=sim_alpha)
            self.positions_ax.scatter(df_frame['sim_x'].iloc[-1], df_frame['sim_y'].iloc[-1], color='black', marker='.', s=100, alpha=sim_alpha, label='_Latest Sim Position')

            self.positions_fig.canvas.draw_idle()

        if len(robot_lines) == 0:
            return

        leg = self.positions_ax.legend(fancybox=True)
        leg.get_frame().set_alpha(0.4)

        for legline in leg.get_lines():
            legline.set_picker(20)

        def on_pick(event: matplotlib.backend_bases.PickEvent):
            impl, robot = event.artist.get_label().split(' ')
            self.get_logger().debug(f'Picked {impl} for {robot}, robot_lines: {robot_lines}')
            self.positions_visibility[robot][impl] = not self.positions_visibility[robot][impl]
            self.positions_fig.canvas.draw_idle()

        if self.cid:
            self.positions_fig.canvas.mpl_disconnect(self.cid)
        self.cid = self.positions_fig.canvas.mpl_connect('pick_event', on_pick)

        t2 = time.time()
        self.get_logger().debug(f'Animating positions took {t2 - t1:.2f} seconds')

    def animate_rmse(self, i):
        pass


def main():
    rclpy.init()
    node = TfLogger(use_pose=True)
    try:
        rclpy.spin(node)
    except:
        node.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
