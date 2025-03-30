# ROS 2 launch file for launching the epuck2 robot in Gazebo and RViz
# Copyright (C) 2024  Jordan Esh

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import os
from launch import LaunchDescription
import launch
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription, GroupAction
import launch_ros
from launch_ros.actions import ComposableNodeContainer, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FileContent
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def get_namespace():
    return (
        launch.substitutions.LaunchConfiguration(
            'manager_robot_tf_prefix'),
        launch.substitutions.LaunchConfiguration(
            'id'),
        launch.substitutions.LaunchConfiguration(
            'manager_robot_tf_suffix'),
    )


def generate_launch_description():
    # Declare launch arguments
    launch_args = list(
        map(
            lambda x: launch.actions.DeclareLaunchArgument(
                name=x[0],
                default_value=x[1],
            ),
            {
                'id': '0',
                'manager_robot_tf_prefix': 'epuck2_robot_',
                'manager_robot_tf_suffix': '',
            }.items(),
        )
    )

    # Launch Gazebo (GUI only)
    robot_localization = launch_ros.actions.Node(
        namespace=get_namespace(),
        package='robot_localization',
        executable='ekf_node',
        parameters=[
            {'use_sim_time': True},
            {
                'frequency': 30.0,
                'sensor_timeout': 0.1,
                'two_d_mode': True,
                'map_frame': [*get_namespace(), '/map'],
                'odom_frame': [*get_namespace(), '/odom'],
                'base_link_frame': [*get_namespace(), '/base_link'],
                'world_frame': [*get_namespace(), '/odom'],
                'imu0': ['/', *get_namespace(), '/imu'],
                'imu0_config': [
                    False, False, False,
                    True, True, True,
                    False, False, False,
                    True, True, True,
                    True, True, True,
                ],
                'odom0': ['/', *get_namespace(), '/odom'],
                'odom0_config': [
                    True, True, True,
                    True, True, True,
                    True, True, True,
                    True, True, True,
                    False, False, False,
                ],
                'acceleration_limits': [0.5, 0.0, 0.0, 0.0, 0.0, 0.0],
                'acceleration_gains': [0.8, 0.0, 0.0, 0.0, 0.0, 0.0],
                'deceleration_limits': [0.5, 0.0, 0.0, 0.0, 0.0, 0.0],
                'deceleration_gains': [0.8, 0.0, 0.0, 0.0, 0.0, 0.0],
            },
        ],
    )

    return LaunchDescription(
        launch_args
        + [
            robot_localization,
        ]
    )
