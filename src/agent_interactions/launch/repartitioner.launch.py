import launch
import launch_ros.actions


def generate_launch_description():
    # Declare launch arguments
    launch_args = list(
        map(
            lambda x: launch.actions.DeclareLaunchArgument(
                name=x[0],
                default_value=x[1],
            ),
            {
                "namespace": "/",
                "robot_id": "0",
                "scheme_name": "vickery_1d",
            }.items(),
        )
    )

    return launch.LaunchDescription(
        launch_args +
        [
            launch_ros.actions.Node(
                namespace=launch.substitutions.LaunchConfiguration(
                    'namespace'),
                package='agent_interactions',
                executable='repartitioner',
                name='agent_interactions',
                output='screen', # ??
                parameters=[
                    {
                        'namespace': launch.substitutions.LaunchConfiguration(
                            "namespace"
                        ),
                        'robot_id': launch.substitutions.LaunchConfiguration(
                            'robot_id'
                        ),
                        'scheme_name': launch.substitutions.LaunchConfiguration(
                            'scheme_name'
                        ),
                    }
                ],
            ),
        ]
    )
