
import os
import launch
import launch_ros.actions

def get_namespace():
    return (
        launch.substitutions.LaunchConfiguration(
            "namespace"),
        launch.substitutions.LaunchConfiguration(
            "manager_robot_tf_prefix"),
        launch.substitutions.LaunchConfiguration(
            "robot_id"),
        launch.substitutions.LaunchConfiguration(
            "manager_robot_tf_suffix"),
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
                "namespace": "/",
                "robot_id": "0",
                "manager_robot_tf_prefix": "epuck2_robot_",
                "manager_robot_tf_suffix": "",
                "manager_robot_tf_frame": "/base_link",
                'min_linear_vel': '0.0',
                'max_linear_vel': '0.1',
                'min_angular_vel': '0.0',
                'max_angular_vel': '1.0',
                'slow_distance': '0.2',
                'slow_angle': '0.2',
                'threshold_distance': '0.05',
                'threshold_angle': '0.05',
            }.items(),
        )
    )

    return launch.LaunchDescription(
        launch_args +
        [
            launch_ros.actions.Node(
                namespace=launch.substitutions.LaunchConfiguration('namespace'),
                package='waypoint_controller',
                executable='controller',
                name='waypoint_controller',
                output='screen',
                parameters=[
                    {
                        'namespace': launch.substitutions.LaunchConfiguration(
                            "namespace"
                        ),
                        'robot_id': launch.substitutions.LaunchConfiguration(
                            'robot_id'
                        ),
                        'manager_robot_tf_prefix': launch.substitutions.LaunchConfiguration(
                            'manager_robot_tf_prefix'
                        ),
                        'manager_robot_tf_suffix': launch.substitutions.LaunchConfiguration(
                            'manager_robot_tf_suffix'
                        ),
                        'manager_robot_tf_frame': launch.substitutions.LaunchConfiguration(
                            'manager_robot_tf_frame'
                        ),
                        'min_linear_vel': launch.substitutions.LaunchConfiguration(
                            'min_linear_vel'
                        ),
                        'max_linear_vel': launch.substitutions.LaunchConfiguration(
                            'max_linear_vel'
                        ),
                        'min_angular_vel': launch.substitutions.LaunchConfiguration(
                            'min_angular_vel'
                        ),
                        'max_angular_vel': launch.substitutions.LaunchConfiguration(
                            'max_angular_vel'
                        ),
                    }
                ],
            ),
        ]
    )
