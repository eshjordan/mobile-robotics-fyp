import launch
import launch_ros
from launch.actions import (
    IncludeLaunchDescription as IncludeLaunch,
    TimerAction,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource as PythonLaunch,
)
from launch.substitutions import (
    PathJoinSubstitution as PathJoin,
)
from launch_ros.substitutions import FindPackageShare


def setup_launch(context):
    # Declare launch arguments
    launch_args = list(
        map(
            lambda x: launch.actions.DeclareLaunchArgument(
                name=x[0],
                default_value=x[1],
            ),
            {
            }.items(),
        )
    )

    return launch_args + [
        launch_ros.actions.Node(
            executable='player',
            package='rosbag2_transport',
            name='rosbag_player',
            parameters=[
                {
                    # 'use_sim_time': True,
                    'play.qos_profile_overrides_path': '/home/jordan/colcon_ws/qos_profiles.yaml',
                    # 'play.start_offset': 15.0,
                    # 'play.start_offset': 13.0,
                    'play.start_paused': True,
                    'play.disable_keyboard_controls': True,
                    # 'play.regex_to_filter': '/tf|/tf_static|.*/mobile_base/cmd_vel|/vrpn_mocap/.*/pose',
                    'play.regex_to_filter': '/tf|/tf_static|/vrpn_mocap/.*/pose|/vrpn_mocap/.*/twist',
                    'storage.uri': '/home/jordan/colcon_ws/rosbag2_2025_05_23-19_35_42'
                }
            ],
            remappings=[
                ('/tf', '/tf_mocap'),
                ('/tf_static', '/tf_static_mocap'),
                # ('/vrpn_mocap/epuck2_robot_5785/twist', '/epuck2_robot_5785/mobile_base/cmd_vel'),
                # ('/vrpn_mocap/epuck2_robot_5653/twist', '/epuck2_robot_5653/mobile_base/cmd_vel'),
                # ('/vrpn_mocap/epuck2_robot_5731/twist', '/epuck2_robot_5731/mobile_base/cmd_vel'),
                # ('/vrpn_mocap/epuck2_robot_5831/twist', '/epuck2_robot_5831/mobile_base/cmd_vel'),
            ]
        ),
    ]


def generate_launch_description():
    """Generate the launch description."""
    return launch.LaunchDescription([
        launch.actions.OpaqueFunction(function=setup_launch)
    ])
