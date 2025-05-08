from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import launch
import launch_ros.actions


def launch_robot_comms() -> list[launch.Action]:
    _node = launch_ros.actions.Node(
        package="agent_local_comms_server",
        executable="udp_agent_comms",
        name=f"knowledge_comms_robot",
        output="screen",
        # prefix=[
        #     # Debugging with gdb
        #     "xterm -bg black -fg white -fa 'Monospace' -fs 13 -e gdb -ex start --args"
        # ],
        # arguments=[
        #     "--ros-args",
        #     "--log-level",
        #     "debug",
        # ],
        parameters=[
            {
                "robot_id": launch.substitutions.LaunchConfiguration(
                    "robot_id"
                ),
                "manager_server_host": launch.substitutions.LaunchConfiguration(
                    "manager_server_host"
                ),
                "manager_server_port": launch.substitutions.LaunchConfiguration(
                    "manager_server_port"
                ),
                "robot_comms_host": launch.substitutions.LaunchConfiguration(
                    "robot_comms_host"
                ),
                "robot_comms_request_port": launch.substitutions.LaunchConfiguration(
                    "robot_comms_request_port"
                ),
                "robot_knowledge_host": launch.substitutions.LaunchConfiguration(
                    "robot_knowledge_host"
                ),
                "robot_knowledge_exchange_port": launch.substitutions.LaunchConfiguration(
                    "robot_knowledge_exchange_port"
                ),
            }
        ],
    )

    return [_node]


def generate_launch_description():
    launch_args = list(
        map(
            lambda x: launch.actions.DeclareLaunchArgument(
                name=x[0],
                default_value=x[1],
            ),
            {
                "robot_id": "0",
                "manager_server_host": "127.0.0.1",
                "manager_server_port": "50000",
                "robot_comms_host": "127.0.0.1",
                "robot_comms_request_port": "50001",
                "robot_knowledge_host": "127.0.0.1",
                "robot_knowledge_exchange_port": "50002",
            }.items(),
        )
    )

    ld = launch.LaunchDescription(
        launch_args
        + launch_robot_comms()
    )
    return ld


if __name__ == "__main__":
    generate_launch_description()
