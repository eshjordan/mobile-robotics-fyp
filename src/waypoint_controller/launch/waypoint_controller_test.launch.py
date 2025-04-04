
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
            }.items(),
        )
    )

    return launch.LaunchDescription(
        launch_args +
        [
            launch_ros.actions.Node(
                namespace=launch.substitutions.LaunchConfiguration("namespace"),
                package='waypoint_controller',
                executable='controller',
                name='waypoint_controller',
                output='screen',
                parameters=[
                    {
                        "robot_id": launch.substitutions.LaunchConfiguration(
                            "robot_id"
                        ),
                        "manager_robot_tf_prefix": launch.substitutions.LaunchConfiguration(
                            "manager_robot_tf_prefix"
                        ),
                        "manager_robot_tf_suffix": launch.substitutions.LaunchConfiguration(
                            "manager_robot_tf_suffix"
                        ),
                        "manager_robot_tf_frame": launch.substitutions.LaunchConfiguration(
                            "manager_robot_tf_frame"
                        ),
                    }
                ],
            ),
        ]
    )
