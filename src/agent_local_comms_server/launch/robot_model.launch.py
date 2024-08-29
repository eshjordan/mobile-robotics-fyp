from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import launch
import launch_ros.actions


def generate_launch_description():
    launch_args = list(
        map(
            lambda x: launch.actions.DeclareLaunchArgument(
                name=x[0],
                default_value=x[1],
            ),
            {
                "manager_server_host": "127.0.0.1",
                "manager_server_port": "50000",
                "manager_threshold_dist": "0.5",
                "manager_robot_tf_prefix": "epuck2_robot_",
                "manager_robot_tf_suffix": "",
                "manager_robot_tf_frame": "base_link",
                "robot0_host": "127.0.0.1",
                "robot0_port": "50002",
                "robot1_host": "127.0.0.1",
                "robot1_port": "50003",
            }.items(),
        )
    )

    ld = launch.LaunchDescription(
        launch_args
        + [
            launch_ros.actions.Node(
                package="agent_local_comms_server",
                executable="test_robot",
                name="test_robot0",
                output="screen",
                parameters=[
                    {
                        "robot_id": 0,
                        "manager_host": launch.substitutions.LaunchConfiguration(
                            "manager_server_host"
                        ),
                        "manager_port": launch.substitutions.LaunchConfiguration(
                            "manager_server_port"
                        ),
                        "robot_host": launch.substitutions.LaunchConfiguration(
                            "robot0_host"
                        ),
                        "robot_port": launch.substitutions.LaunchConfiguration(
                            "robot0_port"
                        ),
                    }
                ],
            ),
            launch_ros.actions.Node(
                package="agent_local_comms_server",
                executable="test_robot",
                name="test_robot1",
                output="screen",
                parameters=[
                    {
                        "robot_id": 1,
                        "manager_host": launch.substitutions.LaunchConfiguration(
                            "manager_server_host"
                        ),
                        "manager_port": launch.substitutions.LaunchConfiguration(
                            "manager_server_port"
                        ),
                        "robot_host": launch.substitutions.LaunchConfiguration(
                            "robot1_host"
                        ),
                        "robot_port": launch.substitutions.LaunchConfiguration(
                            "robot1_port"
                        ),
                    }
                ],
            ),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("agent_local_comms_server"),
                            "launch",
                            "agent_local_comms_server.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "server_host": launch.substitutions.LaunchConfiguration(
                        "manager_server_host"
                    ),
                    "server_port": launch.substitutions.LaunchConfiguration(
                        "manager_server_port"
                    ),
                    "threshold_dist": launch.substitutions.LaunchConfiguration(
                        "manager_threshold_dist"
                    ),
                    "robot_tf_prefix": launch.substitutions.LaunchConfiguration(
                        "manager_robot_tf_prefix"
                    ),
                    "robot_tf_suffix": launch.substitutions.LaunchConfiguration(
                        "manager_robot_tf_suffix"
                    ),
                    "robot_tf_frame": launch.substitutions.LaunchConfiguration(
                        "manager_robot_tf_frame"
                    ),
                }.items(),
            ),
        ]
    )
    return ld


if __name__ == "__main__":
    generate_launch_description()
