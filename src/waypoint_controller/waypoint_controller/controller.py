import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.time
import geometry_msgs.msg
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
import tf2_py
import tf2_ros

# import tf_transformations

import math as mth
from math import sqrt, pi
import numpy as np

# Robot 5653 Z rotation offset: -135.240 degrees, -2.360 rad


class WaypointController_v1(Node):

    def __init__(self):
        super().__init__("waypoint_controller")

        self.declare_parameter("namespace", "~")
        self.declare_parameter("robot_id", 0)
        self.declare_parameter("manager_robot_tf_prefix", "epuck2_robot_")
        self.declare_parameter("manager_robot_tf_suffix", "")
        self.declare_parameter("manager_robot_tf_frame", "/base_link")
        self.declare_parameter("min_linear_vel", 0.0)
        self.declare_parameter("max_linear_vel", 0.1)
        self.declare_parameter("min_angular_vel", 0.0)
        self.declare_parameter("max_angular_vel", 1.0)
        self.declare_parameter("slow_distance", 0.2)
        self.declare_parameter("slow_angle", 0.2)
        self.declare_parameter("threshold_distance", 0.05)
        self.declare_parameter("threshold_angle", 0.05)
        self.declare_parameter("angular_offset", 0.0)

        self.get_logger().info(f"Namespace: {self.get_parameter('namespace').value}")
        self.get_logger().info(
            f"max_angular_vel: {self.get_parameter('max_angular_vel').value}"
        )
        self.get_logger().info(
            f"angular_offset: {self.get_parameter('angular_offset').value}"
        )

        use_tf = True

        if use_tf:
            self.tf_buffer = tf2_ros.buffer.Buffer()
            self.tf_listener = tf2_ros.transform_listener.TransformListener(
                self.tf_buffer, self
            )
            self.update_pose_timer = self.create_timer(1.0 / 60.0, self.update_pose)
        else:
            self.subscriber = self.create_subscription(
                PoseStamped,
                # '/vrpn_mocap/BW_epuck1/pose',
                self.get_parameter("namespace").value + "/pose",
                self.listener_callback,
                qos_profile=QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10,
                ),
            )
            self.subscriber  # prevent unused variable warning

        self.cmd_pub = self.create_publisher(
            geometry_msgs.msg.Twist,
            "mobile_base/cmd_vel",
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
            self.generate_initial_waypoints(), logger=self.get_logger()
        )

        self.current_waypoint = self.path_planner.get_next_waypoint()
        # self.get_logger().info(f"self.current_waypoint {self.current_waypoint}")
        self.prnt_msg = 0

    def update_pose(self):
        robot_frame = (
            self.get_parameter("manager_robot_tf_prefix").value
            + str(self.get_parameter("robot_id").value)
            + self.get_parameter("manager_robot_tf_suffix").value
            + self.get_parameter("manager_robot_tf_frame").value
        )

        try:
            tf = self.tf_buffer.lookup_transform(
                "earth", robot_frame, rclpy.time.Time()
            )
        except tf2_py.LookupException as e:
            self.get_logger().error(f"LookupException: {e}", throttle_duration_sec=5.0)
            return
        except tf2_py.ConnectivityException as e:
            self.get_logger().error(
                f"ConnectivityException: {e}", throttle_duration_sec=5.0
            )
            return
        except tf2_py.ExtrapolationException as e:
            self.get_logger().error(
                f"ExtrapolationException: {e}", throttle_duration_sec=5.0
            )
            return

        # Extract robot pose from /tf
        self.x = tf.transform.translation.x
        self.y = tf.transform.translation.y
        self.z = tf.transform.translation.z

        # Extract robot orientation from /tf
        orientation_q = tf.transform.rotation
        _, _, self.theta = self.euler_from_quaternion(orientation_q)
        self.theta -= self.get_parameter("angular_offset").value

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
        self.theta -= self.get_parameter("angular_offset").value

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

        angle_diff = (angle_to_waypoint - self.theta) % (2 * np.pi)
        # self.get_logger().info(f"angle_diff (1) {angle_diff}\n")

        if angle_diff > np.pi:
            angle_diff -= 2 * np.pi
        # self.get_logger().info(f"angle_diff (2) {angle_diff}\n")

        # angle_diff = ((angle_to_waypoint - self.theta) % 2*np.pi) - np.pi
        # angle_diff = angle_to_waypoint - self.theta
        # angle_diff = mth.atan2(mth.sin(angle_diff), mth.cos(angle_diff))  # Normalize angle to [-pi, pi]

        linear_velocity = 0.0
        angular_velocity = 0.0

        if distance <= self.get_parameter("threshold_distance").value:
            # stop robot
            return linear_velocity, angular_velocity

        if distance <= self.get_parameter("slow_distance").value:
            # slow down linearly between min and max velocity
            linear_velocity_range = (
                self.get_parameter("max_linear_vel").value
                - self.get_parameter("min_linear_vel").value
            )
            linear_closeness = distance / self.get_parameter("slow_distance").value
            linear_velocity = (
                linear_velocity_range * linear_closeness
                + self.get_parameter("min_linear_vel").value
            )
        else:
            linear_velocity = self.get_parameter("max_linear_vel").value

        if abs(angle_diff) <= self.get_parameter("slow_angle").value:
            angular_velocity_range = (
                self.get_parameter("max_angular_vel").value
                - self.get_parameter("min_angular_vel").value
            )
            angular_closeness = abs(angle_diff) / self.get_parameter("slow_angle").value
            angular_velocity = (
                angle_diff / abs(angle_diff) if abs(angle_diff) > 0.001 else 0
            ) * (
                angular_velocity_range * angular_closeness
                + self.get_parameter("min_angular_vel").value
            )
        else:
            angular_velocity = (
                angle_diff / abs(angle_diff) if abs(angle_diff) > 0.001 else 0
            ) * self.get_parameter("max_angular_vel").value

        return linear_velocity, angular_velocity

    def send_twist_message(self):
        """Calcualte twist message to send using current pose and next waypoint"""
        # waypoint = self.path_planner.get_next_waypoint()
        waypoint = [0.0, 0.0]
        if waypoint is None:
            # self.stop_robot()
            return

        # self.current_waypoint = [0, 1]

        distance_to_waypoint = np.linalg.norm(
            np.array(self.current_waypoint) - np.array([self.x, self.y])
        )
        dx = self.current_waypoint[0] - self.x
        dy = self.current_waypoint[1] - self.y
        angle_to_waypoint = mth.atan2(dy, dx)
        angle_diff = (angle_to_waypoint - self.theta) % (2 * np.pi)
        if angle_diff > np.pi:
            angle_diff -= 2 * np.pi
        if distance_to_waypoint < 0.05:
            self.get_logger().info(
                f"Reached waypoint {self.current_waypoint[0]:.2f}, {self.current_waypoint[1]:.2f}"
            )
            self.current_waypoint = self.path_planner.get_next_waypoint()
            self.get_logger().info(
                f"Moving to new waypoint {self.current_waypoint[0]:.2f}, {self.current_waypoint[1]:.2f}"
            )

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

        # self.get_logger().info(
        #     f"self.current_waypoint {self.current_waypoint[0]:.2f}, {self.current_waypoint[1]:.2f}",
        #     throttle_duration_sec=1.0,
        # )
        # self.get_logger().info(
        #     f"Dist to wp: {distance_to_waypoint}", throttle_duration_sec=1.0
        # )
        # self.get_logger().info(
        #     "x: {:7.3f}  y: {:7.3f}".format(self.x, self.y), throttle_duration_sec=1.0
        # )
        # self.get_logger().info(
        #     "angle_to_waypoint: {:7.3f}    robot_angle: {:7.3f}    angle_diff: {:7.3f}".format(
        #         angle_to_waypoint, self.theta, angle_diff
        #     ),
        #     throttle_duration_sec=1.0,
        # )

        # self.get_logger().info(f"sending forward twist command: linear.x = {linear_velocity}, angular.z = {angular_velocity}")

    def stop_robot(self):
        """Stop the robot."""
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.get_logger().info("Robot stopped for interaction")

    def generate_initial_waypoints(self):
        """Generate initial waypoints for the robot to follow."""
        self.get_logger().info(f"Generating initial waypoints for robot {self.get_parameter("robot_id").value}")
        initial_waypoints = generate_boustrophedon_waypoints_radial(
            start_angle=0.0,
            end_angle=2 * np.pi / 4,
            line_spacing=0.1,
            radius=1,
            center=(0, 0),
        )

        log_str = "initial_waypoints:\n"
        for waypoint in initial_waypoints:
            log_str += f"({waypoint[0]:.2f}, {waypoint[1]:.2f})\n"
        self.get_logger().info(log_str)
        return initial_waypoints

    def turn_around(self):
        """Turn the robot around before following new waypoints."""
        turn_angle = pi  # 180 degrees
        self.send_twist_message(0.0, turn_angle)
        self.get_logger().info("Turning around after interaction")

    # def control_loop(self):
    #     """Main control loop for the robot."""
    #     current_position = (self.x, self.y)
    #     current_orientation = self.theta

    #     # Plan the next movement
    #     self.path_planner.plan(current_position, current_orientation)
    #     other_robot_info = None

    #     if other_robot_info:  # if detect other robots
    #         other_x, other_y, other_theta = other_robot_info

    #         # Example: Check if another robot is close
    #         distance_to_other = sqrt((other_x - self.x) ** 2 + (other_y - self.y) ** 2)
    #         if distance_to_other < 2.0:  # Adjust threshold as necessary
    #             self.stop_robot()

    #             # Get other robot's partition info
    #             other_start_angle, other_end_angle = self.get_other_robot_partition()
    #             other_agent_number = self.get_other_robot_agent_number()

    #             # Call your algorithm with both robots' information
    #             self.path_planner.algorithm(
    #                 self.start_angle,
    #                 self.end_angle,
    #                 self.agent_number,
    #                 # Assuming the other robot knows about 2 agents
    #                 other_start_angle,
    #                 other_end_angle,
    #                 other_agent_number,
    #             )
    #             self.turn_around()
    #         else:
    #             self.send_twist_message()
    #     else:
    #         self.send_twist_message()


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


# def generate_boustrophedon_waypoints(
#     start_angle,
#     end_angle,
#     line_spacing=0.1,
#     radius=1,
#     center=(0, 0),
#     boundary_resolution=500,
# ):
#     """
#     Generate boustrophedon-style waypoints across a circular sector,
#     but only using points on the edge of the sector.


#     Parameters:
#     - start_angle (float): start angle of the sector (radians)
#     - end_angle (float): end angle of the sector (radians)
#     - line_spacing (float): vertical spacing between sweep lines
#     - radius (float): radius of the sector
#     - center (tuple): center (x, y) of the sector
#     - boundary_resolution (int): resolution for sampling arc and radial edges


#     Returns:
#     - waypoints (list of (x, y)): only edge-intersection waypoints in boustrophedon order
#     """

#     cx, cy = center

#     # Sample the arc boundary
#     arc_angles = np.linspace(start_angle, end_angle, boundary_resolution)
#     arc_points = [
#         (cx + radius * np.cos(a), cy + radius * np.sin(a)) for a in arc_angles
#     ]

#     # Sample the two radial edges
#     radial_r = np.linspace(0, radius, boundary_resolution)
#     radial1 = [
#         (cx + r * np.cos(start_angle), cy + r * np.sin(start_angle)) for r in radial_r
#     ]
#     radial2 = [
#         (cx + r * np.cos(end_angle), cy + r * np.sin(end_angle)) for r in radial_r
#     ]

#     # Combine all boundary edges
#     boundary_edges = [radial1, arc_points, radial2]
#     all_boundary_points = np.vstack(boundary_edges)

#     # Find vertical extent
#     ys = all_boundary_points[:, 1]
#     min_y = max(min(ys), cy - radius)
#     max_y = min(max(ys), cy + radius)

#     y_vals = np.arange(min_y, max_y + line_spacing, line_spacing)
#     waypoints = []
#     direction = 1

#     for y in y_vals:
#         intersections = []

#         # Check each boundary segment for intersection with this y-level
#         for segment in boundary_edges:
#             for i in range(len(segment) - 1):
#                 (x1, y1), (x2, y2) = segment[i], segment[i + 1]
#                 if (y1 - y) * (y2 - y) <= 0 and y1 != y2:
#                     # Linear interpolation to get intersection x
#                     t = (y - y1) / (y2 - y1)
#                     x = x1 + t * (x2 - x1)
#                     intersections.append((x, y))

#         if len(intersections) >= 2:
#             # Get left and right edges of this sweep line
#             sorted_pts = sorted(intersections)
#             p1, p2 = sorted_pts[0], sorted_pts[-1]
#             if direction % 2 == 0:
#                 waypoints.append(p2)
#                 waypoints.append(p1)
#             else:
#                 waypoints.append(p1)
#                 waypoints.append(p2)
#             direction += 1

#     return waypoints




def generate_boustrophedon_waypoints_polygon(all_boundary_points, line_spacing=0.1):
    """
    Generate boustrophedon-style waypoints across a polygon.

    Args:
        all_boundary_points (np.ndarray): Array of (x,y) points defining the polygon boundary vertices
        line_spacing (float, optional): Vertical spacing between sweep lines (default=0.1)

    Returns:
        waypoints: List of (x,y) tuples representing waypoints in boustrophedon order
    """

    # Find vertical extent
    ys = all_boundary_points[:, 1]
    min_y = min(ys)
    max_y = max(ys)

    y_vals = np.arange(min_y, max_y + line_spacing, line_spacing)
    waypoints = []
    direction = 1

    for y in y_vals:
        intersections = []

        # Check each boundary segment for intersection with this y-level
        for i in range(len(all_boundary_points)):
            (x1, y1), (x2, y2) = all_boundary_points[i,:], all_boundary_points[(i + 1)%len(all_boundary_points),:]
            if (y1 - y) * (y2 - y) <= 0 and y1 != y2:   # check y_val lies inbetween points and points have different y values themselves 
                # Linear interpolation to get intersection x
                t = (y - y1) / (y2 - y1)
                x = x1 + t * (x2 - x1)
                if x not in [itx[0] for itx in intersections]:  # check x value not already in there (handles case where intersection is a vertex)
                    intersections.append((x, y))

        if len(intersections) >= 2:
            # Get left and right edges of this sweep line
            sorted_pts = sorted(intersections)
            p1, p2 = sorted_pts[0], sorted_pts[1]
            if direction % 2 == 0:
                waypoints.append(p2)
                waypoints.append(p1)
            else:
                waypoints.append(p1)
                waypoints.append(p2)
            direction += 1

    return waypoints



def generate_boustrophedon_waypoints_radial(
    start_angle,
    end_angle,
    line_spacing=0.1,
    radius=1,
    center=(0, 0),
    boundary_resolution=500,
):
    """
    Generate boustrophedon-style waypoints across a circular sector,
    but only using points on the edge of the sector.


    Parameters:
    - start_angle (float): start angle of the sector (radians)
    - end_angle (float): end angle of the sector (radians)
    - line_spacing (float): vertical spacing between sweep lines
    - radius (float): radius of the sector
    - center (tuple): center (x, y) of the sector
    - boundary_resolution (int): resolution for sampling arc and radial edges

    Returns:
    - waypoints (list of (x, y)): only edge-intersection waypoints in boustrophedon order
    """

    cx, cy = center

    # Sample the arc boundary
    arc_angles = np.linspace(start_angle, end_angle, boundary_resolution)
    arc_points = [
        (cx + radius * np.cos(a), cy + radius * np.sin(a)) for a in arc_angles
    ]

    boundary_points = [center] + arc_points
    all_boundary_points = np.vstack(boundary_points)

    return generate_boustrophedon_waypoints_polygon(all_boundary_points, line_spacing)





class SimplePathPlanner:
    def __init__(
        self, waypoints, logger=rclpy.logging.get_logger("simple_path_planner")
    ):
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        self.logger = logger
        self.direction = 1  # 1 for forward, -1 for reverse

    def get_next_waypoint(self):
        # Check if we need to reverse direction
        if self.current_waypoint_index == len(self.waypoints) - 1:
            self.direction = -1
        elif self.current_waypoint_index == 0:
            self.direction = 1

        # Update index
        self.current_waypoint_index += self.direction
        return self.waypoints[self.current_waypoint_index]

    def update_waypoints(self, new_waypoints):
        self.waypoints = new_waypoints
        self.current_waypoint_index = 0

    # def algorithm(self, agent1, agent2):
    #     """
    #     TODO: Is this algorithm run centrally ???

    #     Generic agent inputs. Can be a dict or a class, whatever.
    #     It just needs to have the attributes required for whichever scheme we are running.

    #     Uncomment whichever one we are using.
    #     We should probably declare these somewhere else so we can utilise the 'test_neighbourhood' method
    #     """

    #     # Modified from Vickery paper to maintain boundaries and repartition within them
    #     mv1d = schemes.Scheme1dModifiedVickery()
    #     mv1d.interact(agent1, agent2, forward=True)

    #     # # Original algorithm from Vickery paper
    #     # v1d = schemes.Scheme1dVickery()
    #     # v1d.interact(agent1, agent2, forward=True)

    #     # # 2d case with generic polygons
    #     # p2d = schemes.Scheme2dPolygons()
    #     # p2d.interact(agent1, agent2)

    # def algorithm(self, my_start_angle, my_end_angle, my_agent_number,
    #               other_start_angle, other_end_angle, other_agent_number):
    #     """
    #     Algorithm placeholder to calculate partitions based on robot interaction.
    #     Parameters:
    #     - my_start_angle, my_end_angle, my_agent_number:  info of this robot.
    #     - other_start_angle, other_end_angle, other_agent_number: info of the detected robot.

    #     implement  partitioning logic based on the two robots' partition and known agents.
    #     """
    #     print("algorithm")
    #     print(
    #         f"My Partition: Start {my_start_angle}, End {my_end_angle}, Agents known: {my_agent_number}")
    #     print(
    #         f"Other Partition: Start {other_start_angle}, End {other_end_angle}, Agents known: {other_agent_number}")

    #     # Placeholder for partitioning logic
    #     new_start_angle, new_end_angle = 0.0, 2 * \
    #         np.pi  # insert stuff here for algorithm

    #     new_waypoints = generate_boustrophedon_waypoints(
    #         new_start_angle, new_end_angle)
    #     print(f"My new_waypoints: {new_waypoints}")

    #     self.update_waypoints(new_waypoints)

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
