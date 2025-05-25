#!/usr/bin/env python3

import asyncio
import os
import geometry_msgs.msg
import rclpy
import rclpy.executors
import rclpy.task
import rclpy.time

import numpy as np
import time

from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from rclpy.qos import QoSProfile
from rclpy.qos import DurabilityPolicy
from rclpy.qos import ReliabilityPolicy
from threading import Thread
from rosbag2_interfaces.srv import (
    Play as PlayRosbag,
    Stop as StopRosbag,
    Resume as ResumeRosbag,
    Pause as PauseRosbag,
    IsPaused as IsPausedRosbag,
)
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.backend_bases

import transforms3d as tf
def quat_string_to_euler(val):
    return " ".join(map(str, tf.euler.quat2euler([float(v[1]) for v in sorted(dict([tuple(a.split('=')) for a in [s.strip() for s in val.split(",")]]).items())])))

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
                reliability=ReliabilityPolicy.BEST_AVAILABLE,
                durability=DurabilityPolicy.BEST_AVAILABLE,
                depth=100,
            )
            self.epuck2_robot_5785_sub = self.create_subscription(geometry_msgs.msg.PoseStamped, '/vrpn_mocap/epuck2_robot_5785/pose', self.handle_epuck2_robot_5785_cb, qos)
            self.epuck2_robot_5653_sub = self.create_subscription(geometry_msgs.msg.PoseStamped, '/vrpn_mocap/epuck2_robot_5653/pose', self.handle_epuck2_robot_5653_cb, qos)
            self.epuck2_robot_5731_sub = self.create_subscription(geometry_msgs.msg.PoseStamped, '/vrpn_mocap/epuck2_robot_5731/pose', self.handle_epuck2_robot_5731_cb, qos)
            self.epuck2_robot_5831_sub = self.create_subscription(geometry_msgs.msg.PoseStamped, '/vrpn_mocap/epuck2_robot_5831/pose', self.handle_epuck2_robot_5831_cb, qos)
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
        self.next_task_id = 0
        self.running_tasks: dict[int, rclpy.task.Task] = {}
        self.log_tf_tmr = self.create_timer(0.1, self.lookup_tfs, autostart=False)

    def handle_epuck2_robot_5785_cb(self, msg: geometry_msgs.msg.PoseStamped):
        self.handle_pose_cb(msg, 'epuck2_robot_5785')

    def handle_epuck2_robot_5653_cb(self, msg: geometry_msgs.msg.PoseStamped):
        self.handle_pose_cb(msg, 'epuck2_robot_5653')

    def handle_epuck2_robot_5731_cb(self, msg: geometry_msgs.msg.PoseStamped):
        self.handle_pose_cb(msg, 'epuck2_robot_5731')

    def handle_epuck2_robot_5831_cb(self, msg: geometry_msgs.msg.PoseStamped):
        self.handle_pose_cb(msg, 'epuck2_robot_5831')

    def handle_pose_cb(self, msg: geometry_msgs.msg.PoseStamped, robot: str):
        self.get_logger().debug(f'Received pose for {robot}: {msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z}')
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
        self.get_logger().debug(f'Logging transforms for {robot} at {current_time.seconds_nanoseconds()}')
        self.get_logger().debug(f'{robot} Sim TF: {tf_sim.transform.translation.x}, {tf_sim.transform.translation.y}, {tf_sim.transform.translation.z}, {tf_sim.transform.rotation.x}, {tf_sim.transform.rotation.y}, {tf_sim.transform.rotation.z}, {tf_sim.transform.rotation.w}')
        self.get_logger().debug(f'{robot} Bag TF: {tf_bag.transform.translation.x}, {tf_bag.transform.translation.y}, {tf_bag.transform.translation.z}, {tf_bag.transform.rotation.x}, {tf_bag.transform.rotation.y}, {tf_bag.transform.rotation.z}, {tf_bag.transform.rotation.w}')
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
        if not self.executor:
            self.get_logger().error('Executor not set, cannot lookup transforms')
            return

        for i in range(len(robots)):
            robot = robots[i]
            self.running_tasks[self.next_task_id] = self.executor.create_task(self.tf_buffer.lookup_transform_async, 'earth', f'{robot}/base_link', rclpy.time.Time())
            tf_task_id = self.next_task_id
            self.next_task_id += 1
            self.running_tasks[self.next_task_id] = self.executor.create_task(self.tf_mocap_buffer.lookup_transform_async, 'earth', f'{robot}/base_link', rclpy.time.Time())
            tf_mocap_task_id = self.next_task_id
            self.next_task_id += 1


            def call_log_transforms(tf_task_id: int, tf_mocap_task_id: int, robot_name: str, current_time: rclpy.time.Time):
                def inner(this_task: rclpy.task.Task):
                    if this_task.cancelled() or this_task.exception():
                        self.get_logger().error(f'Task failed: {this_task.exception()}')
                        rclpy.shutdown()
                        return

                    if tf_task_id not in self.running_tasks or this_task == self.running_tasks[tf_task_id]:
                        return

                    tf_task = self.running_tasks.pop(tf_task_id)
                    tf_mocap_task = self.running_tasks.pop(tf_mocap_task_id)

                    if tf_task.executing():
                        asyncio.wait_for(tf_task, timeout=0.1)

                    tf_sim = tf_task.result()
                    tf_bag = tf_mocap_task.result()

                    if tf_sim is None:
                        self.get_logger().error(f'tf_sim for {robot_name} is None')

                    if tf_bag is None:
                        self.get_logger().error(f'tf_bag for {robot_name} is None')

                    if tf_sim is None or tf_bag is None:
                        return

                    self.log_transforms(tf_sim, tf_bag, robot_name, current_time)
                return inner

            self.running_tasks[tf_task_id].add_done_callback(call_log_transforms(int(tf_task_id), int(tf_mocap_task_id), f"{robot}", current_time))
            self.running_tasks[tf_mocap_task_id].add_done_callback(call_log_transforms(int(tf_task_id), int(tf_mocap_task_id), f"{robot}", current_time))


class TfPlotter(Node):
    def __init__(self, logger: TfLogger):
        super().__init__('tf_plotter')
        self.logger = logger

        self.declare_parameter('data_dir', '')

        plt.ion()

        self.positions_cid = None
        self.positions_visibility = {}
        self.positions_fig = plt.figure(1)
        self.positions_ax = self.positions_fig.gca()

        self.rmse_cid = None
        self.rmse_visibility = {}
        self.rmse_fig = plt.figure(2)
        self.rmse_ax = self.rmse_fig.gca()

        self.do_show = True
        async def show_positions():
            self.animate_positions(0)
            self.positions_fig.canvas.draw()
            self.positions_fig.canvas.flush_events()
        async def show_rmse():
            self.animate_rmse(0)
            self.rmse_fig.canvas.draw()
            self.rmse_fig.canvas.flush_events()
        def on_show_pos_finish(future):
            self.show_positions_task = rclpy.get_global_executor().create_task(show_positions())
            self.show_positions_task.add_done_callback(on_show_pos_finish)
        def on_show_rmse_finish(future):
            self.show_rmse_task = rclpy.get_global_executor().create_task(show_rmse())
            self.show_rmse_task.add_done_callback(on_show_rmse_finish)

        self.show_positions_task = rclpy.get_global_executor().create_task(show_positions())
        self.show_positions_task.add_done_callback(on_show_pos_finish)
        self.show_rmse_task = rclpy.get_global_executor().create_task(show_rmse())
        self.show_rmse_task.add_done_callback(on_show_rmse_finish)

        self._save_tmr = self.create_timer(2.0, self.save, autostart=False)

        self.data_dir = self.get_parameter('data_dir').get_parameter_value().string_value
        if self.data_dir != '':
            os.makedirs(self.data_dir, exist_ok=True)
            self.get_logger().info(f'Saving data to {self.data_dir}')
            self._save_tmr.reset()

    def __del__(self):
        self.shutdown()

    def animate_positions(self, i):
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

        robot_lines = []

        df = self.logger.df.copy(deep=False)

        for robot in robots:
            if robot not in df['robot'].values:
                continue

            df_frame = df[df['robot'] == robot]

            if robot not in self.positions_visibility:
                self.positions_visibility[robot] = {'Mocap': True, 'Sim': True}

            robot_lines += [robot]

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
            legline.set_picker(10)

        def on_pick(event: matplotlib.backend_bases.PickEvent):
            impl, robot = event.artist.get_label().split(' ')
            self.get_logger().debug(f'Picked {impl} for {robot}, robot_lines: {robot_lines}')
            self.positions_visibility[robot][impl] = not self.positions_visibility[robot][impl]
            self.positions_fig.canvas.draw_idle()

        if self.positions_cid:
            self.positions_fig.canvas.mpl_disconnect(self.positions_cid)
        self.positions_cid = self.positions_fig.canvas.mpl_connect('pick_event', on_pick)

        t2 = time.time()
        self.get_logger().debug(f'Animating positions took {t2 - t1:.2f} seconds')

    def animate_rmse(self, i):
        t1 = time.time()
        self.rmse_ax.clear()
        self.rmse_ax.set_title('TF RMSE')
        self.rmse_ax.set_xlabel('Time (s)')
        self.rmse_ax.set_ylabel('RMSE (m)')
        self.rmse_ax.grid()

        # Show +-5m range
        self.rmse_ax.set_ylim(0, 8)

        robot_lines = []

        df = self.logger.df.copy(deep=False)

        for robot in robots:
            if robot not in df['robot'].values:
                continue

            df_frame = df[df['robot'] == robot]

            if robot not in self.rmse_visibility:
                self.rmse_visibility[robot] = True

            robot_lines += [robot]

            rmse_alpha = 1.0 if self.rmse_visibility[robot] else 0.0
            x_vals = df_frame['diff_x'].to_numpy(np.float32)
            y_vals = df_frame['diff_y'].to_numpy(np.float32)
            z_vals = df_frame['diff_z'].to_numpy(np.float32)

            rmse = np.sqrt(x_vals**2 + y_vals**2 + z_vals**2)
            time_series = df_frame['timestamp']

            self.rmse_ax.plot(time_series, rmse, color=robot_colors[robot], label=robot, alpha=rmse_alpha)
            self.rmse_fig.canvas.draw_idle()

        if len(robot_lines) == 0:
            return

        leg = self.rmse_ax.legend(fancybox=True)
        leg.get_frame().set_alpha(0.4)

        for legline in leg.get_lines():
            legline.set_picker(10)

        def on_pick(event: matplotlib.backend_bases.PickEvent):
            robot = event.artist.get_label()
            self.get_logger().debug(f'Picked RMSE for {robot}, robot_lines: {robot_lines}')
            self.rmse_visibility[robot] = not self.rmse_visibility[robot]
            self.rmse_fig.canvas.draw_idle()

        if self.rmse_cid:
            self.rmse_fig.canvas.mpl_disconnect(self.rmse_cid)
        self.rmse_cid = self.rmse_fig.canvas.mpl_connect('pick_event', on_pick)

        t2 = time.time()
        self.get_logger().debug(f'Animating RMSE took {t2 - t1:.2f} seconds')


    def shutdown(self):
        self.do_show = False
        self.get_logger().warning('Shutting down tf logger')
        # self.save()
        plt.ioff()
        plt.close(self.positions_fig)
        plt.close(self.rmse_fig)

    def save(self):
        self.positions_fig.savefig(os.path.join(self.data_dir, 'tf_log_positions.png'))
        self.rmse_fig.savefig(os.path.join(self.data_dir, 'tf_log_rmse.png'))
        self.logger.df.to_csv(os.path.join(self.data_dir, 'tf_log.csv'), index=False)

    def show(self):
        if self.do_show:
            async def show_positions():
                self.animate_positions(0)
                self.positions_fig.canvas.draw()
                self.positions_fig.canvas.flush_events()
            async def show_rmse():
                self.animate_rmse(0)
                self.rmse_fig.canvas.draw()
                self.rmse_fig.canvas.flush_events()
            t1 = time.time()
            task1 = rclpy.get_global_executor().create_task(show_positions())
            task2 = rclpy.get_global_executor().create_task(show_rmse())



class TwistRepub(Node):
    def __init__(self):
        super().__init__('twist_repub')
        self.get_logger().info('Starting twist republisher')

        self.epuck2_robot_5785_sub = self.create_subscription(geometry_msgs.msg.TwistStamped, '/vrpn_mocap/epuck2_robot_5785/twist', self.epuck2_robot_5785_cb, qos_profile=QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10,
        ))
        self.epuck2_robot_5785_pub = self.create_publisher(geometry_msgs.msg.Twist, '/epuck2_robot_5785/mobile_base/cmd_vel', 10)
        self.epuck2_robot_5653_sub = self.create_subscription(geometry_msgs.msg.TwistStamped, '/vrpn_mocap/epuck2_robot_5653/twist', self.epuck2_robot_5653_cb, qos_profile=QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10,
        ))
        self.epuck2_robot_5653_pub = self.create_publisher(geometry_msgs.msg.Twist, '/epuck2_robot_5653/mobile_base/cmd_vel', 10)
        self.epuck2_robot_5731_sub = self.create_subscription(geometry_msgs.msg.TwistStamped, '/vrpn_mocap/epuck2_robot_5731/twist', self.epuck2_robot_5731_cb, qos_profile=QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10,
        ))
        self.epuck2_robot_5731_pub = self.create_publisher(geometry_msgs.msg.Twist, '/epuck2_robot_5731/mobile_base/cmd_vel', 10)
        self.epuck2_robot_5831_sub = self.create_subscription(geometry_msgs.msg.TwistStamped, '/vrpn_mocap/epuck2_robot_5831/twist', self.epuck2_robot_5831_cb, qos_profile=QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10,
        ))
        self.epuck2_robot_5831_pub = self.create_publisher(geometry_msgs.msg.Twist, '/epuck2_robot_5831/mobile_base/cmd_vel', 10)

    def handle_cmd_vel(self, msg: geometry_msgs.msg.TwistStamped, robot_name: str):
        self.get_logger().debug(f'Received cmd_vel: {msg.twist.linear.x}, {msg.twist.linear.y}, {msg.twist.angular.z}')
        new_msg = geometry_msgs.msg.Twist()
        new_msg.linear.x = msg.twist.linear.x
        new_msg.linear.y = msg.twist.linear.y
        new_msg.linear.z = msg.twist.linear.z
        new_msg.angular.x = msg.twist.angular.x
        new_msg.angular.y = msg.twist.angular.y
        new_msg.angular.z = msg.twist.angular.z
        if robot_name == 'epuck2_robot_5785':
            self.epuck2_robot_5785_pub.publish(new_msg)
        elif robot_name == 'epuck2_robot_5653':
            self.epuck2_robot_5653_pub.publish(new_msg)
        elif robot_name == 'epuck2_robot_5731':
            self.epuck2_robot_5731_pub.publish(new_msg)
        elif robot_name == 'epuck2_robot_5831':
            self.epuck2_robot_5831_pub.publish(new_msg)

    def epuck2_robot_5785_cb(self, msg: geometry_msgs.msg.TwistStamped):
        self.handle_cmd_vel(msg, 'epuck2_robot_5785')
    def epuck2_robot_5653_cb(self, msg: geometry_msgs.msg.TwistStamped):
        self.handle_cmd_vel(msg, 'epuck2_robot_5653')
    def epuck2_robot_5731_cb(self, msg: geometry_msgs.msg.TwistStamped):
        self.handle_cmd_vel(msg, 'epuck2_robot_5731')
    def epuck2_robot_5831_cb(self, msg: geometry_msgs.msg.TwistStamped):
        self.handle_cmd_vel(msg, 'epuck2_robot_5831')


def main():
    rclpy.init()
    executor = rclpy.executors.SingleThreadedExecutor()
    node = TfLogger(use_pose=True)
    plotter = TfPlotter(node)
    twist_repub = TwistRepub()
    executor.add_node(node)
    executor.add_node(twist_repub)
    node_thread = Thread(target=executor.spin)
    node_thread.start()

    try:
        rclpy.spin(plotter)
    except Exception as e:
        node.shutdown()
        rclpy.shutdown()
        raise e


if __name__ == '__main__':
    main()
