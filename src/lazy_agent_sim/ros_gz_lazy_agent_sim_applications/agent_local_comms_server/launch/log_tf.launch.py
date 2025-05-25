import launch
import launch_ros
from launch.actions import (
    IncludeLaunchDescription as IncludeLaunch,
    TimerAction,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource as PythonLaunch,
)
from launch.substitutions import (
    PathJoinSubstitution as PathJoin,
)
from launch_ros.substitutions import FindPackageShare

def setup_launch(context):
    # Declare launch arguments
    launch_args = list(
        map(
            lambda x: launch.actions.DeclareLaunchArgument(
                name=x[0],
                default_value=x[1],
            ),
            {
                'data_dir': '',
            }.items(),
        )
    )

    return launch_args + [
        IncludeLaunch(
            PythonLaunch(
                PathJoin(
                    [
                        FindPackageShare(
                            'agent_local_comms_server',
                        ),
                        'launch',
                        'bag.launch.py',
                    ]
                )
            ),
            launch_arguments={}.items(),
        ),
        TimerAction(
            period=4.0,
            actions=[
                launch_ros.actions.Node(
                    executable='log_tf',
                    package='agent_local_comms_server',
                    parameters=[
                        {
                            # 'use_sim_time': True,
                            'data_dir': launch.substitutions.LaunchConfiguration('data_dir'),
                        }
                    ],
                ),
            ],
        ),
    ]


def generate_launch_description():
    """Generate the launch description."""
    return launch.LaunchDescription([
        launch.actions.OpaqueFunction(function=setup_launch)
    ])
