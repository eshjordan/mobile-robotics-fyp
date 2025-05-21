#!/usr/bin/env python3

gz_topics = """
/clock
/epuck2_robot_5653/camera
/epuck2_robot_5653/dist_sens
/epuck2_robot_5653/imu
/epuck2_robot_5653/joint_states
/epuck2_robot_5653/mobile_base/cmd_vel
/epuck2_robot_5653/odom
/epuck2_robot_5653/pose
/epuck2_robot_5653/proximity0
/epuck2_robot_5653/proximity1
/epuck2_robot_5653/proximity2
/epuck2_robot_5653/proximity3
/epuck2_robot_5653/proximity4
/epuck2_robot_5653/proximity5
/epuck2_robot_5653/proximity6
/epuck2_robot_5653/proximity7
/epuck2_robot_5653/robot_description
/epuck2_robot_5653/scan
/epuck2_robot_5731/camera
/epuck2_robot_5731/dist_sens
/epuck2_robot_5731/imu
/epuck2_robot_5731/joint_states
/epuck2_robot_5731/mobile_base/cmd_vel
/epuck2_robot_5731/odom
/epuck2_robot_5731/pose
/epuck2_robot_5731/proximity0
/epuck2_robot_5731/proximity1
/epuck2_robot_5731/proximity2
/epuck2_robot_5731/proximity3
/epuck2_robot_5731/proximity4
/epuck2_robot_5731/proximity5
/epuck2_robot_5731/proximity6
/epuck2_robot_5731/proximity7
/epuck2_robot_5731/robot_description
/epuck2_robot_5731/scan
/epuck2_robot_5785/camera
/epuck2_robot_5785/dist_sens
/epuck2_robot_5785/imu
/epuck2_robot_5785/joint_states
/epuck2_robot_5785/mobile_base/cmd_vel
/epuck2_robot_5785/odom
/epuck2_robot_5785/pose
/epuck2_robot_5785/proximity0
/epuck2_robot_5785/proximity1
/epuck2_robot_5785/proximity2
/epuck2_robot_5785/proximity3
/epuck2_robot_5785/proximity4
/epuck2_robot_5785/proximity5
/epuck2_robot_5785/proximity6
/epuck2_robot_5785/proximity7
/epuck2_robot_5785/robot_description
/epuck2_robot_5785/scan
/epuck2_robot_5831/camera
/epuck2_robot_5831/dist_sens
/epuck2_robot_5831/imu
/epuck2_robot_5831/joint_states
/epuck2_robot_5831/mobile_base/cmd_vel
/epuck2_robot_5831/odom
/epuck2_robot_5831/pose
/epuck2_robot_5831/proximity0
/epuck2_robot_5831/proximity1
/epuck2_robot_5831/proximity2
/epuck2_robot_5831/proximity3
/epuck2_robot_5831/proximity4
/epuck2_robot_5831/proximity5
/epuck2_robot_5831/proximity6
/epuck2_robot_5831/proximity7
/epuck2_robot_5831/robot_description
/epuck2_robot_5831/scan
/parameter_events
/rosout
/tf
/tf_static
"""

bag_topics = """
/agent_local_comms_server/interaction
/agent_local_comms_server/knowledge
/agent_local_comms_server/repartition
/clicked_point
/epuck2_robot_5653/battery
/epuck2_robot_5653/dist_sens
/epuck2_robot_5653/imu
/epuck2_robot_5653/joint_states
/epuck2_robot_5653/mag_field
/epuck2_robot_5653/mag_field_vector
/epuck2_robot_5653/microphone
/epuck2_robot_5653/mobile_base/cmd_vel
/epuck2_robot_5653/odom
/epuck2_robot_5653/proximity0
/epuck2_robot_5653/proximity1
/epuck2_robot_5653/proximity2
/epuck2_robot_5653/proximity3
/epuck2_robot_5653/proximity4
/epuck2_robot_5653/proximity5
/epuck2_robot_5653/proximity6
/epuck2_robot_5653/proximity7
/epuck2_robot_5653/robot_description
/epuck2_robot_5653/scan
/epuck2_robot_5731/joint_states
/epuck2_robot_5731/mobile_base/cmd_vel
/epuck2_robot_5731/robot_description
/epuck2_robot_5785/battery
/epuck2_robot_5785/dist_sens
/epuck2_robot_5785/imu
/epuck2_robot_5785/joint_states
/epuck2_robot_5785/mag_field
/epuck2_robot_5785/mag_field_vector
/epuck2_robot_5785/microphone
/epuck2_robot_5785/mobile_base/cmd_vel
/epuck2_robot_5785/odom
/epuck2_robot_5785/proximity0
/epuck2_robot_5785/proximity1
/epuck2_robot_5785/proximity2
/epuck2_robot_5785/proximity3
/epuck2_robot_5785/proximity4
/epuck2_robot_5785/proximity5
/epuck2_robot_5785/proximity6
/epuck2_robot_5785/proximity7
/epuck2_robot_5785/robot_description
/epuck2_robot_5785/scan
/epuck2_robot_5831/battery
/epuck2_robot_5831/dist_sens
/epuck2_robot_5831/imu
/epuck2_robot_5831/joint_states
/epuck2_robot_5831/mag_field
/epuck2_robot_5831/mag_field_vector
/epuck2_robot_5831/microphone
/epuck2_robot_5831/mobile_base/cmd_vel
/epuck2_robot_5831/odom
/epuck2_robot_5831/proximity0
/epuck2_robot_5831/proximity1
/epuck2_robot_5831/proximity2
/epuck2_robot_5831/proximity3
/epuck2_robot_5831/proximity4
/epuck2_robot_5831/proximity5
/epuck2_robot_5831/proximity6
/epuck2_robot_5831/proximity7
/epuck2_robot_5831/robot_description
/epuck2_robot_5831/scan
/events/write_split
/initialpose
/move_base_simple/goal
/parameter_events
/repartition/reset
/rosout
/tf
/tf_static
/vrpn_mocap/epuck2_robot_5653/accel
/vrpn_mocap/epuck2_robot_5653/pose
/vrpn_mocap/epuck2_robot_5653/twist
/vrpn_mocap/epuck2_robot_5731/accel
/vrpn_mocap/epuck2_robot_5731/pose
/vrpn_mocap/epuck2_robot_5731/twist
/vrpn_mocap/epuck2_robot_5785/accel
/vrpn_mocap/epuck2_robot_5785/pose
/vrpn_mocap/epuck2_robot_5785/twist
/vrpn_mocap/epuck2_robot_5831/accel
/vrpn_mocap/epuck2_robot_5831/pose
/vrpn_mocap/epuck2_robot_5831/twist
"""


# gz_topics
# bag_topics

gz_topics = gz_topics.strip().split("\n")
bag_topics = bag_topics.strip().split("\n")

import transforms3d as tf

# ([x, y], [qx, qy, qz, qw])
epuck2_robot_5785 = ([1.642464, -1.560504], [0.018296, -0.008551, 0.157211, 0.987359])
epuck2_robot_5653 = ([1.605983, 0.51829], [0.013822, 0.00051, -0.103538, 0.994529])
epuck2_robot_5731 = ([-0.440772, -1.151986], [0.023069, 0.026405, 0.08028, 0.996155])
epuck2_robot_5831 = ([-1.192578, 0.045757], [0.00617, 0.000338, -0.722181, 0.691676])

print(epuck2_robot_5785[0], tf.euler.quat2euler([epuck2_robot_5785[1][-1]] + epuck2_robot_5785[1][:-1]))
print(epuck2_robot_5653[0], tf.euler.quat2euler([epuck2_robot_5653[1][-1]] + epuck2_robot_5653[1][:-1]))
print(epuck2_robot_5731[0], tf.euler.quat2euler([epuck2_robot_5731[1][-1]] + epuck2_robot_5731[1][:-1]))
print(epuck2_robot_5831[0], tf.euler.quat2euler([epuck2_robot_5831[1][-1]] + epuck2_robot_5831[1][:-1]))
