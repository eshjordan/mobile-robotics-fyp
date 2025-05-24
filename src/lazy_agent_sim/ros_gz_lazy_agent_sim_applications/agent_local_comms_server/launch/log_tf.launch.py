import launch
import launch_ros


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
                    'play.start_offset': 10.0,
                    'play.start_paused': True,
                    'play.disable_keyboard_controls': True,
                    'play.regex_to_filter': '/tf|/tf_static|.*/mobile_base/cmd_vel|/vrpn_mocap/.*/pose',
                    'storage.uri': '/home/jordan/colcon_ws/rosbag2_2025_05_17-18_59_00'
                }
            ],
            remappings=[
                ('/tf', '/tf_mocap'),
                ('/tf_static', '/tf_static_mocap'),
            ]
        ),
        launch_ros.actions.Node(
            executable='log_tf',
            package='agent_local_comms_server',
            name='log_tf',
            parameters=[
                {
                    # 'use_sim_time': True,
                }
            ]
        ),
    ]


def generate_launch_description():
    """Generate the launch description."""
    return launch.LaunchDescription([
        launch.actions.OpaqueFunction(function=setup_launch)
    ])
