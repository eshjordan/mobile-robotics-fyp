
import os
import launch
import launch_ros.actions
  
def generate_launch_description():
   return launch.LaunchDescription([
       launch_ros.actions.Node(
           package='waypoint_controller',
           executable='controller',
           name='waypoint_controller',
           output='screen',
           parameters=[
               {
                   'linear': {'x' : 0.0, 'y' : 0.0, 'z' : 0.0},
                   'angular': {'x' : 0.0, 'y' : 0.0, 'z' : 0.0},
               },
           ]
       ),


   ])

