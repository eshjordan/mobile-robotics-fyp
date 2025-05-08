"""Launchfile for the cpp_epuck package."""

import launch

import launch_ros.actions


def generate_launch_description():
    """Generate the main launch description."""
    launch_args = [
        launch.actions.DeclareLaunchArgument(name=x[0], default_value=x[1])
        for x in {
            'robot_id': '0',
            'manager_host': '127.0.0.1',
            'manager_port': '50000',
            'robot_comms_host': '127.0.0.1',
            'robot_comms_request_port': '50001',
            'robot_knowledge_host': '127.0.0.1',
            'robot_knowledge_exchange_port': '50002',
        }.items()
    ]

    ld = launch.LaunchDescription(
        launch_args
        + [
            launch_ros.actions.Node(
                package='cpp_epuck',
                executable='cpp_epuck',
                name='knowledge_comms_robot',
                output='screen',
                parameters=[
                    {
                        'robot_id': launch.substitutions.LaunchConfiguration(
                            'robot_id'
                        ),
                        'manager_host': launch.substitutions
                        .LaunchConfiguration(
                            'manager_host'
                        ),
                        'manager_port': launch.substitutions
                        .LaunchConfiguration(
                            'manager_port'
                        ),
                        'robot_comms_host': launch.substitutions
                        .LaunchConfiguration(
                            'robot_comms_host'
                        ),
                        'robot_comms_request_port': launch.substitutions
                        .LaunchConfiguration(
                            'robot_comms_request_port'
                        ),
                        'robot_knowledge_host': launch.substitutions
                        .LaunchConfiguration(
                            'robot_knowledge_host'
                        ),
                        'robot_knowledge_exchange_port': launch.substitutions
                        .LaunchConfiguration(
                            'robot_knowledge_exchange_port'
                        ),
                    }
                ],
            ),
        ],
    )

    return ld


if __name__ == '__main__':
    generate_launch_description()
