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
                'source_topic_name': '',
                'source_frame_id': 'earth',
                'source_child_frame_id': 'base_link',
                'tf_frame_id': 'map',
                'tf_child_frame_id': 'odom',
                'pose_topic_name': '',
                'pose_frame_id': 'earth',
                'pose_child_frame_id': 'odom',
            }.items(),
        )
    )

    return launch_args + [
        launch_ros.actions.Node(
            executable='pose_tf',
            package='agent_local_comms_server',
            name='pose_tf',
            parameters=[
                {
                    'source_topic_name': launch.substitutions.LaunchConfiguration('source_topic_name'),
                    'source_frame_id': launch.substitutions.LaunchConfiguration('source_frame_id'),
                    'source_child_frame_id': launch.substitutions.LaunchConfiguration('source_child_frame_id'),
                    'tf_frame_id': launch.substitutions.LaunchConfiguration('tf_frame_id'),
                    'tf_child_frame_id': launch.substitutions.LaunchConfiguration('tf_child_frame_id'),
                    'pose_topic_name': launch.substitutions.LaunchConfiguration('pose_topic_name'),
                    'pose_frame_id': launch.substitutions.LaunchConfiguration('pose_frame_id'),
                    'pose_child_frame_id': launch.substitutions.LaunchConfiguration('pose_child_frame_id'),
                }
            ]
        ),
    ]


def generate_launch_description():
    """Generate the launch description."""
    return launch.LaunchDescription([
        launch.actions.OpaqueFunction(function=setup_launch)
    ])
