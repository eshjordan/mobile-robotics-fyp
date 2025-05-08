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
import tempfile
from launch import LaunchDescription
import launch
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.actions import IncludeLaunchDescription, GroupAction
import launch_ros
from launch_ros.actions import ComposableNodeContainer, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FileContent, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


ROBOT_CONFIG_TEMPLATE = [
    {
        "ros_topic_name": "{}/mobile_base/cmd_vel",
        "gz_topic_name": "/model{}/cmd_vel",
        "ros_type_name": "geometry_msgs/msg/Twist",
        "gz_type_name": "gz.msgs.Twist",
        "direction": "ROS_TO_GZ",
    },
    {
        "ros_topic_name": "{}/pose",
        "gz_topic_name": "/model{}/pose",
        "ros_type_name": "geometry_msgs/msg/PoseStamped",
        "gz_type_name": "gz.msgs.Pose",
        "direction": "GZ_TO_ROS",
    },
    {
        "ros_topic_name": "{}/odom",
        "gz_topic_name": "/model{}/odometry",
        "ros_type_name": "nav_msgs/msg/Odometry",
        "gz_type_name": "gz.msgs.Odometry",
        "direction": "GZ_TO_ROS",
    },
    {
        "ros_topic_name": "{}/joint_states",
        "gz_topic_name": "/world/earth/model{}/joint_state",
        "ros_type_name": "sensor_msgs/msg/JointState",
        "gz_type_name": "gz.msgs.Model",
        "direction": "GZ_TO_ROS",
    },
    {
        "ros_topic_name": "/tf",
        "gz_topic_name": "/model{}/tf",
        "ros_type_name": "tf2_msgs/msg/TFMessage",
        "gz_type_name": "gz.msgs.Pose_V",
        "direction": "GZ_TO_ROS",
    },
    {
        "ros_topic_name": "/tf_static",
        "gz_topic_name": "/model{}/pose_static",
        "ros_type_name": "tf2_msgs/msg/TFMessage",
        "gz_type_name": "gz.msgs.Pose_V",
        "direction": "GZ_TO_ROS",
    },
    {
        "ros_topic_name": "{}/imu",
        "gz_topic_name": "/world/earth/model{}/link/base_link/sensor/imu/imu",
        "ros_type_name": "sensor_msgs/msg/Imu",
        "gz_type_name": "gz.msgs.IMU",
        "direction": "GZ_TO_ROS",
    },
    {
        "ros_topic_name": "{}/proximity0",
        "gz_topic_name": "/world/earth/model{}/link/base_link/sensor/prox0/scan",
        "ros_type_name": "sensor_msgs/msg/LaserScan",
        "gz_type_name": "gz.msgs.LaserScan",
        "direction": "GZ_TO_ROS",
    },
    {
        "ros_topic_name": "{}/scan",
        "gz_topic_name": "/world/earth/model{}/link/base_link/sensor/prox0/scan",
        "ros_type_name": "sensor_msgs/msg/LaserScan",
        "gz_type_name": "gz.msgs.LaserScan",
        "direction": "GZ_TO_ROS",
    },
    {
        "ros_topic_name": "{}/proximity1",
        "gz_topic_name": "/world/earth/model{}/link/base_link/sensor/prox1/scan",
        "ros_type_name": "sensor_msgs/msg/LaserScan",
        "gz_type_name": "gz.msgs.LaserScan",
        "direction": "GZ_TO_ROS",
    },
    {
        "ros_topic_name": "{}/scan",
        "gz_topic_name": "/world/earth/model{}/link/base_link/sensor/prox1/scan",
        "ros_type_name": "sensor_msgs/msg/LaserScan",
        "gz_type_name": "gz.msgs.LaserScan",
        "direction": "GZ_TO_ROS",
    },
    {
        "ros_topic_name": "{}/proximity2",
        "gz_topic_name": "/world/earth/model{}/link/base_link/sensor/prox2/scan",
        "ros_type_name": "sensor_msgs/msg/LaserScan",
        "gz_type_name": "gz.msgs.LaserScan",
        "direction": "GZ_TO_ROS",
    },
    {
        "ros_topic_name": "{}/scan",
        "gz_topic_name": "/world/earth/model{}/link/base_link/sensor/prox2/scan",
        "ros_type_name": "sensor_msgs/msg/LaserScan",
        "gz_type_name": "gz.msgs.LaserScan",
        "direction": "GZ_TO_ROS",
    },
    {
        "ros_topic_name": "{}/proximity3",
        "gz_topic_name": "/world/earth/model{}/link/base_link/sensor/prox3/scan",
        "ros_type_name": "sensor_msgs/msg/LaserScan",
        "gz_type_name": "gz.msgs.LaserScan",
        "direction": "GZ_TO_ROS",
    },
    {
        "ros_topic_name": "{}/scan",
        "gz_topic_name": "/world/earth/model{}/link/base_link/sensor/prox3/scan",
        "ros_type_name": "sensor_msgs/msg/LaserScan",
        "gz_type_name": "gz.msgs.LaserScan",
        "direction": "GZ_TO_ROS",
    },
    {
        "ros_topic_name": "{}/proximity4",
        "gz_topic_name": "/world/earth/model{}/link/base_link/sensor/prox4/scan",
        "ros_type_name": "sensor_msgs/msg/LaserScan",
        "gz_type_name": "gz.msgs.LaserScan",
        "direction": "GZ_TO_ROS",
    },
    {
        "ros_topic_name": "{}/scan",
        "gz_topic_name": "/world/earth/model{}/link/base_link/sensor/prox4/scan",
        "ros_type_name": "sensor_msgs/msg/LaserScan",
        "gz_type_name": "gz.msgs.LaserScan",
        "direction": "GZ_TO_ROS",
    },
    {
        "ros_topic_name": "{}/proximity5",
        "gz_topic_name": "/world/earth/model{}/link/base_link/sensor/prox5/scan",
        "ros_type_name": "sensor_msgs/msg/LaserScan",
        "gz_type_name": "gz.msgs.LaserScan",
        "direction": "GZ_TO_ROS",
    },
    {
        "ros_topic_name": "{}/scan",
        "gz_topic_name": "/world/earth/model{}/link/base_link/sensor/prox5/scan",
        "ros_type_name": "sensor_msgs/msg/LaserScan",
        "gz_type_name": "gz.msgs.LaserScan",
        "direction": "GZ_TO_ROS",
    },
    {
        "ros_topic_name": "{}/proximity6",
        "gz_topic_name": "/world/earth/model{}/link/base_link/sensor/prox6/scan",
        "ros_type_name": "sensor_msgs/msg/LaserScan",
        "gz_type_name": "gz.msgs.LaserScan",
        "direction": "GZ_TO_ROS",
    },
    {
        "ros_topic_name": "{}/scan",
        "gz_topic_name": "/world/earth/model{}/link/base_link/sensor/prox6/scan",
        "ros_type_name": "sensor_msgs/msg/LaserScan",
        "gz_type_name": "gz.msgs.LaserScan",
        "direction": "GZ_TO_ROS",
    },
    {
        "ros_topic_name": "{}/proximity7",
        "gz_topic_name": "/world/earth/model{}/link/base_link/sensor/prox7/scan",
        "ros_type_name": "sensor_msgs/msg/LaserScan",
        "gz_type_name": "gz.msgs.LaserScan",
        "direction": "GZ_TO_ROS",
    },
    {
        "ros_topic_name": "{}/scan",
        "gz_topic_name": "/world/earth/model{}/link/base_link/sensor/prox7/scan",
        "ros_type_name": "sensor_msgs/msg/LaserScan",
        "gz_type_name": "gz.msgs.LaserScan",
        "direction": "GZ_TO_ROS",
    },
    {
        "ros_topic_name": "{}/dist_sens",
        "gz_topic_name": "/world/earth/model{}/link/base_link/sensor/tof/scan",
        "ros_type_name": "sensor_msgs/msg/LaserScan",
        "gz_type_name": "gz.msgs.LaserScan",
        "direction": "GZ_TO_ROS",
    },
    {
        "ros_topic_name": "{}/scan",
        "gz_topic_name": "/world/earth/model{}/link/base_link/sensor/tof/scan",
        "ros_type_name": "sensor_msgs/msg/LaserScan",
        "gz_type_name": "gz.msgs.LaserScan",
        "direction": "GZ_TO_ROS",
    },
    {
        "ros_topic_name": "{}/camera",
        "gz_topic_name": "/world/earth/model{}/link/base_link/sensor/camera/image",
        "ros_type_name": "sensor_msgs/msg/Image",
        "gz_type_name": "gz.msgs.Image",
        "direction": "GZ_TO_ROS",
    },
]


def launch_static_transforms(context) -> list[launch.Action]:
    """Launch static transforms for the robots."""

    result = []

    for i, agent in enumerate(launch_configuration['agents']):
        arguments = []
        for k, v in transform.items():
            arguments.append(f'--{k}')
            if 'frame' in k:
                arguments.append(
                    f"{v.format(launch_configuration['manager_robot_tf_prefix'] + str(agent['robot_id']) + launch_configuration['manager_robot_tf_suffix'])}"
                )
            elif k == 'x':
                arguments.append(
                    f"{v.format(agent['robot_xpos'])}"
                )
            elif k == 'y':
                arguments.append(
                    f"{v.format(agent['robot_ypos'])}"
                )
            elif k == 'yaw':
                arguments.append(
                    f"{v.format(agent['robot_theta'])}"
                )
            else:
                arguments.append(v)

        _static_transform = launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'static_agent_transform_{i}',
            output='screen',
            arguments=arguments,
        )

        result.append(_static_transform)

    return result


def setup_launch(context):
    # Declare launch arguments
    launch_args = list(
        map(
            lambda x: launch.actions.DeclareLaunchArgument(
                name=x[0],
                default_value=x[1],
            ),
            {
                'robot_ids': '',
                "gz_version": "8",
                "gui": "true",
                "manager_robot_tf_prefix": "epuck2_robot_",
                "manager_robot_tf_suffix": "",
            }.items(),
        )
    ) + [
        SetEnvironmentVariable(
            "GZ_VERSION", LaunchConfiguration("gz_version")),
    ]

    # Find paths to other projects
    sim_bringup = FindPackageShare("ros_gz_lazy_agent_sim_bringup")
    sim_gazebo = FindPackageShare("ros_gz_lazy_agent_sim_gazebo")
    ros_gz_sim = FindPackageShare("ros_gz_sim")

    # Launch Gazebo (GUI only)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim, "launch", "gz_sim.launch.py"]),
        ),
        launch_arguments={"gz_args": "-g"}.items(),
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    def get_namespace(robot_id: int):
        return (
            launch.substitutions.LaunchConfiguration(
                "manager_robot_tf_prefix").perform(context)
            + f"{robot_id}"
            + launch.substitutions.LaunchConfiguration(
                "manager_robot_tf_suffix").perform(context)
        )

    class BridgeConfigBuilder:
        def __init__(self):
            self.config = []

        def add_bridge(self, ros_topic_name, gz_topic_name, ros_type_name, gz_type_name, direction):
            self.config.append({
                "ros_topic_name": ros_topic_name,
                "gz_topic_name": gz_topic_name,
                "ros_type_name": ros_type_name,
                "gz_type_name": gz_type_name,
                "direction": direction,
            })

        def build(self):
            fp = tempfile.NamedTemporaryFile(delete=False)
            with open(fp.name, 'w') as f:
                f.write("---\n")
                for entry in self.config:
                    f.write(f"- ros_topic_name: {entry['ros_topic_name']}\n")
                    f.write(f"  gz_topic_name: {entry['gz_topic_name']}\n")
                    f.write(f"  ros_type_name: {entry['ros_type_name']}\n")
                    f.write(f"  gz_type_name: {entry['gz_type_name']}\n")
                    f.write(f"  direction: {entry['direction']}\n")
            return fp.name

    config_builder = BridgeConfigBuilder()

    # Add the clock bridge
    config_builder.add_bridge(
        "/clock",
        "/clock",
        "rosgraph_msgs/msg/Clock",
        "gz.msgs.Clock",
        "GZ_TO_ROS",
    )

    def process_robot_ids(context):
        robot_ids_value = LaunchConfiguration("robot_ids").perform(context)
        return robot_ids_value.split(",") if robot_ids_value else []

    # TODO: Make models in gazebo world configurable from launchfile
    robot_ids = process_robot_ids(context)
    gz_world_robot_ids = [0, 1, 2, 3]
    frame_publishers = []

    for i in range(min(len(robot_ids), len(gz_world_robot_ids))):
        robot_id = robot_ids[i]
        gz_world_robot_id = gz_world_robot_ids[i]

        for entry in ROBOT_CONFIG_TEMPLATE:
            config_builder.add_bridge(
                entry["ros_topic_name"].format(f"/{get_namespace(robot_id)}"),
                entry["gz_topic_name"].format(
                    f"/epuck2_robot_{gz_world_robot_id}"),
                entry["ros_type_name"],
                entry["gz_type_name"],
                entry["direction"],
            )

        group = GroupAction([
            PushRosNamespace(get_namespace(robot_id)),
            launch_ros.actions.Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {"use_sim_time": True},
                    {
                        "frame_prefix": get_namespace(robot_id) + "/",
                        "publish_frequency": 60.0,
                        "robot_description": FileContent(
                            [
                                PathJoinSubstitution(
                                    [
                                        FindPackageShare(
                                            "ros_gz_lazy_agent_sim_description"
                                        ),
                                        "models",
                                        "epuck2",
                                        "epuck2.urdf",
                                    ]
                                ),
                            ]
                        ),
                    },
                ],
            ),
            # Create static transforms to wrap the gazebo-named frames
            launch_ros.actions.Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='tf_pub_gz_odom',
                output='screen',
                condition=IfCondition(PythonExpression(
                    f"'{get_namespace(robot_id)}/odom' != 'epuck2_robot_{gz_world_robot_id}/odom'")),
                arguments=[
                    '--frame-id',
                    f"{get_namespace(robot_id)}/odom",
                    '--child-frame-id',
                    f"epuck2_robot_{gz_world_robot_id}/odom",
                ],
            ),
            launch_ros.actions.Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='tf_pub_gz_base_link',
                output='screen',
                condition=IfCondition(PythonExpression(
                    f"'epuck2_robot_{gz_world_robot_id}/base_link' != '{get_namespace(robot_id)}/base_link'")),
                arguments=[
                    '--frame-id',
                    f"epuck2_robot_{gz_world_robot_id}/base_link",
                    '--child-frame-id',
                    f"{get_namespace(robot_id)}/base_link",
                ],
            )
        ])

        frame_publishers.append(group)

    # Bridge ROS topics and Gazebo messages for establishing communication
    parameter_bridge = ComposableNode(
        package="ros_gz_bridge",
        plugin="ros_gz_bridge::RosGzBridge",
        name="parameter_bridge",
        parameters=[
            {
                "use_sim_time": True,
                "expand_gz_topic_names": True,
                # "config_file": PathJoinSubstitution(
                #     [sim_bringup, "config", "ros_gz_lazy_agent_sim_bridge.yaml"]
                # ),
                "config_file": config_builder.build(),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # Launch Gazebo server
    gz_server = \
        ComposableNode(
            package="ros_gz_sim",
            plugin="ros_gz_sim::GzServer",
            name="gz_server",
            parameters=[
                {
                    "world_sdf_file": PathJoinSubstitution(
                        [sim_gazebo, "worlds", "epuck2.sdf"]
                    ),
                }
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )

    # Run the bridge and Gazebo server in a container, better communication
    gz_container = ComposableNodeContainer(
        name="gz_bridged_sim",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            parameter_bridge,
            gz_server,
        ],
        ros_arguments=['--disable-stdout-logs'],
    )

    return launch_args + [
        gz_sim,
        gz_container,
    ] + frame_publishers


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=setup_launch)])
