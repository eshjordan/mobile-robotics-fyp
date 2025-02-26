import launch
import launch.actions
import launch.launch_description_sources
import launch.substitutions

import launch_ros
import launch_ros.actions
import launch_ros.substitutions

from launch.logging import get_output_loggers


def launch_waypoint_controller(id: int):
    _controller = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            launch.substitutions.PathJoinSubstitution(
                [
                    launch_ros.substitutions.FindPackageShare("waypoint_controller"),
                    "launch",
                    "main.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_id": f"{id}",
        }.items(),
    )

    return [_controller]


def generate_launch_description():
    launch_args = list(
        map(
            lambda x: launch.actions.DeclareLaunchArgument(
                name=x[0],
                default_value=x[1],
            ),
            {
                # "robot_id": "0",
            }.items(),
        )
    )

    waypoint_controller_nodes = []

    ld = launch.LaunchDescription(
        launch_args
        + launch_waypoint_controller(0)
        + launch_waypoint_controller(1)
        + launch_waypoint_controller(2)
        + launch_waypoint_controller(3)
        + [
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                    launch.substitutions.PathJoinSubstitution(
                        [
                            launch_ros.substitutions.FindPackageShare(
                                "ros_gz_lazy_agent_sim_bringup"
                            ),
                            "launch",
                            "epuck2.launch.py",
                        ]
                    )
                ),
            ),
        ]
    )

    return ld
