import rclpy
from rclpy.node import Node
import geometry_msgs.msg
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
import tf2_ros
# import tf_transformations

import math as mth
from math import sqrt, pi
import numpy as np

# Robot 5653 Z rotation offset: -135.240 degrees, -2.360 rad


class WaypointController_v1(Node):

    def __init__(self):
        super().__init__('waypoint_controller')

        self.declare_parameter('namespace', '~')
        self.declare_parameter('robot_id', 0)
        self.declare_parameter('manager_robot_tf_prefix', 'epuck2_robot_')
        self.declare_parameter('manager_robot_tf_suffix', '')
        self.declare_parameter('manager_robot_tf_frame', '/base_link')
        self.declare_parameter('min_linear_vel', 0.0)
        self.declare_parameter('max_linear_vel', 0.1)
        self.declare_parameter('min_angular_vel', 0.0)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('slow_distance', 0.2)
        self.declare_parameter('slow_angle', 0.2)
        self.declare_parameter('threshold_distance', 0.05)
        self.declare_parameter('threshold_angle', 0.05)
        self.declare_parameter('angular_offset', 0.0)

        self.get_logger().info(
            f"Namespace: {self.get_parameter('namespace').value}")
        self.get_logger().info(
            f"max_angular_vel: {self.get_parameter('max_angular_vel').value}")
        self.get_logger().info(
            f"angular_offset: {self.get_parameter('angular_offset').value}")

        self.subscriber = self.create_subscription(
            PoseStamped,
            '/vrpn_mocap/BW_epuck1/pose',
            # self.get_parameter('namespace').value + '/pose',
            self.listener_callback,
            qos_profile=QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            ))
        self.subscriber  # prevent unused variable warning

        self.cmd_pub = self.create_publisher(
            geometry_msgs.msg.Twist,
            'mobile_base/cmd_vel',
            10,
        )

        # Robot current velocity twist msgs
        # send message every second
        self.create_timer(0.01, self.send_twist_message)
        # Robot's curernt pose and orientation
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.theta = 0.0

        # Robot's partition information
        self.start_angle = 0.0
        self.end_angle = 2 * np.pi
        self.agent_number = 1
        # Path planner instance
        # self.get_logger().info("SimplePathPlanner")
        # triggered when robot interact or when robot reaches its waypoint, or
        self.path_planner = SimplePathPlanner(
            self.generate_initial_waypoints())

        self.current_waypoint = self.path_planner.get_next_waypoint()
        # self.get_logger().info(f"self.current_waypoint {self.current_waypoint}")
        self.prnt_msg = 0

    def listener_callback(self, msg):
        # print(msg.x, msg.y, msg.z)
        # self.get_logger().info(f"subscribing vicon position = {msg.pose.position}")

        # print(msg.pose.position)
        # print("\n\n\n")
        # print(msg.pose)

        # Extract pose position from vicon topic
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z

        # Extract orientation from vicon topic using quaternion
        orientation_q = msg.pose.orientation
        # self.get_logger().info(f"orientation_q = {orientation_q}")
        _, _, self.theta = self.euler_from_quaternion(orientation_q)
        self.theta -= self.get_parameter('angular_offset').value

        # self.prnt_msg += 1
        # if self.prnt_msg % 100 == 0:
        #     self.get_logger().info("x: {:10.3f}   y: {:10.3f}   z: {:10.3f}   theta: {:10.3f}".format(self.x, self.y, self.z, self.theta))

        # print(msg)
        # self.get_logger().info

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        # self.get_logger().info(f"yaw = {yaw}")
        return roll, pitch, yaw

    def calculate_velocities(self, waypoint):
        """Calculate the linear and angular velocities to reach the next waypoint."""
        # ## Hardcode pose - comment out when using vicon
        # self.x = 1.0
        # self.y = 0.0

        if self.current_waypoint is None:
            self.stop_robot()
            return

        # calculate velocities
        dx = self.current_waypoint[0] - self.x
        dy = self.current_waypoint[1] - self.y
        # print(f"self,x {self.x}, self.y {self.y}")

        distance = mth.sqrt(dx**2 + dy**2)
        # print("distance", distance)

        angle_to_waypoint = mth.atan2(dy, dx)
        # self.get_logger().info(f"angle_to_waypoint {angle_to_waypoint}\n")

        angle_diff = ((angle_to_waypoint - self.theta) % (2*np.pi))
        # self.get_logger().info(f"angle_diff (1) {angle_diff}\n")

        if angle_diff > np.pi:
            angle_diff -= 2*np.pi
        # self.get_logger().info(f"angle_diff (2) {angle_diff}\n")

        # angle_diff = ((angle_to_waypoint - self.theta) % 2*np.pi) - np.pi
        # angle_diff = angle_to_waypoint - self.theta
        # angle_diff = mth.atan2(mth.sin(angle_diff), mth.cos(angle_diff))  # Normalize angle to [-pi, pi]

        linear_velocity = 0.0
        angular_velocity = 0.0

        if distance <= self.get_parameter('threshold_distance').value:
            # stop robot
            return linear_velocity, angular_velocity

        if distance <= self.get_parameter('slow_distance').value:
            # slow down linearly between min and max velocity
            linear_velocity_range = self.get_parameter(
                'max_linear_vel').value - self.get_parameter('min_linear_vel').value
            linear_closeness = distance / \
                self.get_parameter('slow_distance').value
            linear_velocity = linear_velocity_range * linear_closeness + \
                self.get_parameter('min_linear_vel').value
        else:
            linear_velocity = self.get_parameter('max_linear_vel').value

        if abs(angle_diff) <= self.get_parameter('slow_angle').value:
            angular_velocity_range = self.get_parameter(
                'max_angular_vel').value - self.get_parameter('min_angular_vel').value
            angular_closeness = abs(angle_diff) / \
                self.get_parameter('slow_angle').value
            angular_velocity = (angle_diff/abs(angle_diff) if abs(angle_diff) > 0.001 else 0) * (
                angular_velocity_range * angular_closeness + self.get_parameter('min_angular_vel').value)
        else:
            angular_velocity = (angle_diff/abs(angle_diff) if abs(angle_diff)
                                > 0.001 else 0) * self.get_parameter('max_angular_vel').value

        return linear_velocity, angular_velocity

    def send_twist_message(self):
        """Calcualte twist message to send using current pose and next waypoint"""
        # waypoint = self.path_planner.get_next_waypoint()
        waypoint = [0.0,
                    0.0]
        if waypoint is None:
            # self.stop_robot()
            return

        # self.current_waypoint = [0, 1]

        distance_to_waypoint = np.linalg.norm(
            np.array(self.current_waypoint) - np.array([self.x, self.y]))
        dx = self.current_waypoint[0] - self.x
        dy = self.current_waypoint[1] - self.y
        angle_to_waypoint = mth.atan2(dy, dx)
        angle_diff = ((angle_to_waypoint - self.theta) % (2*np.pi))
        if angle_diff > np.pi:
            angle_diff -= 2*np.pi
        if distance_to_waypoint < 0.05:
            self.get_logger().info(f"Reached waypoint {self.current_waypoint}")
            self.current_waypoint = self.path_planner.get_next_waypoint()
            self.get_logger().info(
                f"Moving to new waypoint {self.current_waypoint}")

        # self.current_waypoint = [0, 1]

        # hardcoded velocities
        # linear_velocity = 0.1
        # angular_velocity = 0.0
        # linear_velocity = 0.05 * distance
        # angular_velocity = 1.0 * angle_diff

        # # use current velocity - dynamic
        # linear_velocity = self.get_parameter('linear.x').value
        # angular_velocity = self.get_parameter('angular.z').value

        linear_velocity, angular_velocity = self.calculate_velocities(waypoint)

        twist = geometry_msgs.msg.Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        self.cmd_pub.publish(twist)

        self.prnt_msg += 1
        if self.prnt_msg % 10 == 0:
            self.get_logger().info(
                f"self.current_waypoint {self.current_waypoint}")
            self.get_logger().info(f"Dist to wp: {distance_to_waypoint}")
            self.get_logger().info("x: {:7.3f}  y: {:7.3f}  z: {:7.3f}  theta: {:7.3f} linear_velocity: {:7.3f}  angular_z: {:7.3} pi".format(
                self.x, self.y, self.z, self.theta, linear_velocity, angular_velocity/np.pi))
            self.get_logger().info("angle_to_waypoint: {:7.3f}    robot_angle: {:7.3f}    angle_diff: {:7.3f}".format(
                angle_to_waypoint, self.theta, angle_diff))

        # self.get_logger().info(f"sending forward twist command: linear.x = {linear_velocity}, angular.z = {angular_velocity}")

    def stop_robot(self):
        """Stop the robot."""
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.get_logger().info('Robot stopped for interaction')

    def generate_initial_waypoints(self):
        """Generate initial waypoints for the robot to follow."""
        self.get_logger().info("generate_initial_waypoints")
        initial_waypoints = generate_boustrophedon_waypoints(
            start_angle=0.0, end_angle=np.pi,
            spacing=0.1, radius=1,
            center=(0, 0),
            num_points=10
        )

        self.get_logger().info(f"initial_waypoints {initial_waypoints}")
        return initial_waypoints

    def turn_around(self):
        """Turn the robot around before following new waypoints."""
        turn_angle = pi  # 180 degrees
        self.send_twist_message(0.0, turn_angle)
        self.get_logger().info('Turning around after interaction')

    def control_loop(self):
        """Main control loop for the robot."""
        current_position = (self.x, self.y)
        current_orientation = self.theta

        # Plan the next movement
        self.path_planner.plan(current_position, current_orientation)
        other_robot_info = None

        if other_robot_info:  # if detect other robots
            other_x, other_y, other_theta = other_robot_info

            # Example: Check if another robot is close
            distance_to_other = sqrt(
                (other_x - self.x)**2 + (other_y - self.y)**2)
            if distance_to_other < 2.0:  # Adjust threshold as necessary
                self.stop_robot()

                # Get other robot's partition info
                other_start_angle, other_end_angle = self.get_other_robot_partition()
                other_agent_number = self.get_other_robot_agent_number()

                # Call your algorithm with both robots' information
                self.path_planner.algorithm(
                    self.start_angle, self.end_angle, self.agent_number,
                    # Assuming the other robot knows about 2 agents
                    other_start_angle, other_end_angle, other_agent_number
                )
                self.turn_around()
            else:
                self.send_twist_message()
        else:
            self.send_twist_message()

#    def get_other_robot_position(self, other_robot_frame):
#         """ Get position and orientation of another robot using tf2. #######################################################################################
#         Args: other_robot_frame (str): The tf frame ID of the other robot.
#         Returns: (float, float, float): The x, y position and orientation (theta) of the other robot."""
#         try:
#             # Lookup transform from the other robot to the base frame
#             transform = self.tf_buffer.lookup_transform(
#                 'base_link', other_robot_frame, rclpy.time.Time())
#             x = transform.transform.translation.x
#             y = transform.transform.translation.y

#             # Convert quaternion to yaw angle (theta)
#             orientation_q = transform.transform.rotation
#             _, _, theta = self.euler_from_quaternion(orientation_q)

#             return x, y, theta
#         except:
#             self.get_logger().warn(f"Could not get transform for {other_robot_frame}")
#             return None

#     def get_other_robot_partition(self):
#         """Placeholder function to get another robot's partition info.""" #######################################################################################
#         return 0.0, np.pi  # Replace with actual logic

#     def get_other_robot_agent_number(self):
#         """Placeholder function to get another robot's knwon agent number info."""#######################################################################################
#         return 2  # Replace with actual logic


def generate_boustrophedon_waypoints(
    start_angle, end_angle,
    spacing=0.1, radius=1,
    center=(0, 0),
    num_points=10
):
    """
    Generate horizontal boustrophedon-style waypoints across a circular sector.

    Parameters:
    - start_angle (float): start angle of sector (radians)
    - end_angle (float): end angle of sector (radians)
    - spacing (float): vertical spacing between sweep lines
    - radius (float): radius of sector
    - center (tuple): (x, y) center of sector
    - num_points (int): how many points to use when defining sector boundary

    Returns:
    - waypoints (list of (x, y)): ordered boustrophedon sweep waypoints
    """

    # Create points along the circular arc (boundary of the sector)
    arc_angles = np.linspace(start_angle, end_angle, num_points)
    arc_points = [(center[0] + radius * np.cos(a), center[1] +
                   radius * np.sin(a)) for a in arc_angles]

    # Get bounding box of the arc
    all_y = [p[1] for p in arc_points]
    min_y = max(min(all_y), center[1] - radius)
    max_y = min(max(all_y), center[1] + radius)

    y_vals = np.arange(min_y, max_y, spacing)
    waypoints = []
    direction = 1

    for y in y_vals:
        x_range = []
        for angle in arc_angles:
            x = center[0] + radius * np.cos(angle)
            y_angle = center[1] + radius * np.sin(angle)
            if abs(y_angle - y) < spacing / 2:
                x_range.append(x)

        if len(x_range) >= 2:
            x_min = min(x_range)
            x_max = max(x_range)
            sweep_line = np.linspace(x_min, x_max, num_points)
            if direction % 2 == 0:
                sweep_line = sweep_line[::-1]
            for x in sweep_line:
                waypoints.append((x, y))
            direction += 1

    return waypoints


class SimplePathPlanner:
    def __init__(self, waypoints):
        self.waypoints = waypoints
        self.current_waypoint_index = 0

    def get_next_waypoint(self):
        if self.current_waypoint_index < len(self.waypoints)-1:
            self.current_waypoint_index += 1
            return self.waypoints[self.current_waypoint_index]
        return self.waypoints[-1]

    def update_waypoints(self, new_waypoints):
        self.waypoints = new_waypoints
        self.current_waypoint_index = 0

    def algorithm(self, my_start_angle, my_end_angle, my_agent_number,
                  other_start_angle, other_end_angle, other_agent_number):
        """
        Algorithm placeholder to calculate partitions based on robot interaction.
        Parameters:
        - my_start_angle, my_end_angle, my_agent_number:  info of this robot.
        - other_start_angle, other_end_angle, other_agent_number: info of the detected robot.

        implement  partitioning logic based on the two robots' partition and known agents.
        """
        print("algorithm")
        print(
            f"My Partition: Start {my_start_angle}, End {my_end_angle}, Agents known: {my_agent_number}")
        print(
            f"Other Partition: Start {other_start_angle}, End {other_end_angle}, Agents known: {other_agent_number}")

        # Placeholder for partitioning logic
        new_start_angle, new_end_angle = 0.0, 2 * \
            np.pi  # insert stuff here for algorithm

        new_waypoints = generate_boustrophedon_waypoints(
            new_start_angle, new_end_angle)
        print(f"My new_waypoints: {new_waypoints}")

        self.update_waypoints(new_waypoints)

    def plan(self, current_position, current_orientation):
        """Plan the robot's path towards the next waypoint."""
        waypoint = self.get_next_waypoint()
        if waypoint is None:
            return  # No more waypoints, stop planning


def main():
    rclpy.init()
    waypoint_controller = WaypointController_v1()
    # Run the control loop
    rclpy.spin(waypoint_controller)
    # # Shutdown
    # waypoint_controller.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    main()
