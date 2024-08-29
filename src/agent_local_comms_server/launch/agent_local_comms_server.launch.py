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
                "server_host": "127.0.0.1",
                "server_port": "50000",
                "threshold_dist": "0.5",
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
                package="agent_local_comms_server",
                executable="agent_local_comms_server",
                name="agent_local_comms_server",
                output="screen",
                parameters=[
                    {
                        "server_host": launch.substitutions.LaunchConfiguration(
                            "server_host"
                        ),
                        "server_port": launch.substitutions.LaunchConfiguration(
                            "server_port"
                        ),
                        "threshold_dist": launch.substitutions.LaunchConfiguration(
                            "threshold_dist"
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


if __name__ == "__main__":
    generate_launch_description()
