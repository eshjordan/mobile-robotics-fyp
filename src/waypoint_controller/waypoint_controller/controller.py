import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import sqrt, atan2, pi
from controller import Robot
from tf2_ros import TransformListener, Buffer

def generate_circumference_waypoints(start_angle, end_angle, radius = 10, num_waypoints = 25 , center = (0,0)):
    """
    Generate waypoints along the circumference of a circular slice.

    Parameters:
    center (tuple): (x, y) center of the circle
    start_angle (float): Starting angle of the slice (in radians)
    end_angle (float): Ending angle of the slice (in radians)
    radius (float): Radius of the circumference
    num_waypoints (int): Number of waypoints along the circumference

    Returns:
    waypoints (list): List of (x, y) waypoints along the circumference
    """

    waypoints = []

    # Generate evenly spaced angles along the arc
    angular_steps = np.linspace(start_angle, end_angle, num_waypoints)

    # Convert each angle to Cartesian coordinates and store as waypoints
    for theta in angular_steps:
        x = center[0] + radius * np.cos(theta)
        y = center[1] + radius * np.sin(theta)
        waypoints.append((x, y))

    return waypoints

class SimplePathPlanner:
    def __init__(self, waypoints):
        self.waypoints = waypoints
        self.current_waypoint_index = 0

    def get_next_waypoint(self):
        if self.current_waypoint_index < len(self.waypoints):
            return self.waypoints[self.current_waypoint_index]
        return None

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
        self.get_logger().info(f"My Partition: Start {my_start_angle}, End {my_end_angle}, Agents known: {my_agent_number}")
        self.get_logger().info(f"Other Partition: Start {other_start_angle}, End {other_end_angle}, Agents known: {other_agent_number}")

        # Placeholder for partitioning logic
        new_start_angle, new_end_angle = 0.0, 2*np.pi####################################################################################### insert stuff here for algorithm
        
        new_waypoints = generate_circumference_waypoints(new_start_angle, new_end_angle)
        self.update_waypoints(new_waypoints)

    def plan(self, current_position, current_orientation):
        """ Plan the robot's path towards the next waypoint."""
        waypoint = self.get_next_waypoint()
        if waypoint is None:
            return  # No more waypoints, stop planning

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')

        # Initialize odometry subscriber
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Initialize publisher for velocity commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # # Initialize tf2 buffer and listener ###################################################################################### uncomment
        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)

        # Robot state variables
        self.x = 0.0  # Current x position
        self.y = 0.0  # Current y position
        self.theta = 0.0  # Current orientation (yaw)

        # Robot's partition information
        self.start_angle = 0.0
        self.end_angle = 2 * np.pi
        self.agent_number = 1

        # Path planner instance
        self.path_planner = SimplePathPlanner(self.generate_initial_waypoints())

        # Initialize the e-puck robot
        self.epuck = EPUCKRobot()

    def odom_callback(self, msg):
        """Callback function to process odometry data."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Extract orientation from quaternion
        orientation_q = msg.pose.pose.orientation
        _, _, self.theta = self.euler_from_quaternion(orientation_q)

    def euler_from_quaternion(self, q):
        """Convert quaternion (x, y, z, w) to euler angles (roll, pitch, yaw)."""
        import tf_transformations
        quaternion = (q.x, q.y, q.z, q.w)
        euler = tf_transformations.euler_from_quaternion(quaternion)
        return euler  # Returns roll, pitch, yaw (we use yaw as theta)

    def get_other_robot_position(self, other_robot_frame):
        """ Get position and orientation of another robot using tf2. #######################################################################################
        Args: other_robot_frame (str): The tf frame ID of the other robot.
        Returns: (float, float, float): The x, y position and orientation (theta) of the other robot."""
        try:
            # Lookup transform from the other robot to the base frame
            transform = self.tf_buffer.lookup_transform(
                'base_link', other_robot_frame, rclpy.time.Time())
            x = transform.transform.translation.x
            y = transform.transform.translation.y

            # Convert quaternion to yaw angle (theta)
            orientation_q = transform.transform.rotation
            _, _, theta = self.euler_from_quaternion(orientation_q)

            return x, y, theta
        except:
            self.get_logger().warn(f"Could not get transform for {other_robot_frame}")
            return None

    def get_other_robot_partition(self):
        """Placeholder function to get another robot's partition info.""" #######################################################################################
        return 0.0, np.pi  # Replace with actual logic

    def get_other_robot_agent_number(self):
        """Placeholder function to get another robot's knwon agent number info."""#######################################################################################
        return 2  # Replace with actual logic

    def control_loop(self):
        """Main control loop for the robot."""
        current_position = (self.x, self.y)
        current_orientation = self.theta

        # Plan the next movement
        self.path_planner.plan(current_position, current_orientation)

        # Check for other robots' positions via tf2 instead of proximity sensors
        # other_robot_frame = 'other_robot_base_link'  # Example frame of the other robot .####################################################################################### uncomment
        # other_robot_info = self.get_other_robot_info(other_robot_frame)
        other_robot_info = None

        if other_robot_info: #if detect other robots
            other_x, other_y, other_theta = other_robot_info

            # Example: Check if another robot is close
            distance_to_other = sqrt((other_x - self.x)**2 + (other_y - self.y)**2)
            if distance_to_other < 2.0:  # Adjust threshold as necessary
                self.stop_robot()

                # Get other robot's partition info
                other_start_angle, other_end_angle = self.get_other_robot_partition()
                other_agent_number = self.get_other_robot_agent_number()

                # Call your algorithm with both robots' information
                self.path_planner.algorithm(
                    self.start_angle, self.end_angle, self.agent_number,
                    other_start_angle, other_end_angle, other_agent_number  # Assuming the other robot knows about 2 agents
                )
                self.turn_around()
            else:
                self.send_twist_message()
        else:
            self.send_twist_message()

    def send_twist_message(self):
        """Continue moving towards the next waypoint if no obstacles are detected."""
        waypoint = self.path_planner.get_next_waypoint()
        if waypoint is None:
            self.stop_robot()
            return

        dx = waypoint[0] - self.x
        dy = waypoint[1] - self.y
        distance = sqrt(dx**2 + dy**2)
        angle_to_waypoint = atan2(dy, dx)
        angle_diff = angle_to_waypoint - self.theta

        linear_velocity = self.k_linear * distance
        angular_velocity = self.k_angular * angle_diff

        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        self.publisher.publish(twist)

    def stop_robot(self):
        """Stop the robot."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.get_logger().info('Robot stopped for interaction')

    def turn_around(self):
        """Turn the robot around before following new waypoints."""
        turn_angle = pi  # 180 degrees
        self.send_twist_message(0.0, turn_angle)
        self.get_logger().info('Turning around after interaction')

    def generate_initial_waypoints(self):
        """Generate initial waypoints for the robot to follow."""
        return generate_circumference_waypoints(
            start_angle=0,
            end_angle=2 * pi,
            radius=10,
            num_waypoints=25,
            center=(0, 0)
        )

def main():

    rclpy.init()
    velocity_controller = VelocityController()

    # Run the control loop
    rclpy.spin(velocity_controller)

    # Shutdown
    velocity_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
