import launch
import launch.actions
import launch.launch_description_sources
import launch.substitutions

import launch_ros
import launch_ros.actions
import launch_ros.substitutions

from launch.logging import get_output_loggers


def generate_launch_description():
    launch_args = list(
        map(
            lambda x: launch.actions.DeclareLaunchArgument(
                name=x[0],
                default_value=x[1],
            ),
            {
                "robot_id": "0",
                "robot_tf_prefix": "epuck2_robot_",
                "robot_tf_suffix": "",
                "robot_tf_frame": "base_link",
            }.items(),
        )
    )

    ld = launch.LaunchDescription(
        launch_args
        + [
            launch_ros.actions.Node(
                package="waypoint_controller",
                executable="controller",
                name="waypoint_controller",
                parameters=[
                    {
                        "robot_id": launch.substitutions.LaunchConfiguration(
                            "robot_id"
                        ),
                        "robot_tf_prefix": launch.substitutions.LaunchConfiguration(
                            "robot_tf_prefix"
                        ),
                        "robot_tf_suffix": launch.substitutions.LaunchConfiguration(
                            "robot_tf_suffix"
                        ),
                        "robot_tf_frame": launch.substitutions.LaunchConfiguration(
                            "robot_tf_frame"
                        ),
                    }
                ],
            ),
        ]
    )

    return ld
