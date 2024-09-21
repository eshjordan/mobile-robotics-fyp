
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt, pi, cos, sin


class VelocityController:
    def __init__(self):

        self.node = rclpy.create_node('velocity_controller')
        self.publisher = self.node.create_publisher(Twist, 'mobile_base/cmd_vel', 10)
        self.odom_subscriber = self.node.create_subscription(
            Odometry,
            'odom',  # Topic name for odometry data
            self.odom_callback,
            10
        )
        self.timer = self.node.create_timer(
            0.1,  # Update frequency in seconds
            self.control_loop
        )

        # Parameters for proportional control
        self.k_linear = 0.5  # Linear velocity gain
        self.k_angular = 1.0  # Angular velocity gain

        # Parameters for stopping criteria
        self.distance_threshold = 0.1  # Threshold distance to consider the robot has reached the target
        self.angle_threshold = 0.05    # Threshold angle to consider the robot is oriented correctly

        # Current position and target position
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.x_target = 1.0
        self.y_target = 1.0

        # Parameters for odometry calculation
        self.wheel_base = 0.05  # Distance between wheels
        self.wheel_radius = 0.02  # Radius of the wheels

    def odom_callback(self, msg):
        """
        Callback function to update the robot's position based on odometry data.
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        _, _, self.theta = self._quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)

    def _quaternion_to_euler(self, x, y, z, w):
        """
        Convert quaternion to Euler angles.
        """
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        theta = atan2(siny_cosp, cosy_cosp)
        return (0.0, 0.0, theta)  # Only theta (yaw) is used

    def control_loop(self):
        # Compute the desired angle and distance
        dx = self.x_target - self.x
        dy = self.y_target - self.y
        distance = sqrt(dx**2 + dy**2)
        angle_to_target = atan2(dy, dx)

        # Compute the angle difference
        angle_diff = angle_to_target - self.theta
        if angle_diff > pi:
            angle_diff -= 2 * pi
        elif angle_diff < -pi:
            angle_diff += 2 * pi

        # Check if the robot is within the stopping criteria
        if distance < self.distance_threshold and abs(angle_diff) < self.angle_threshold:
            # Stop the robot
            linear_velocity = 0.0
            angular_velocity = 0.0
        else:
            # Compute linear and angular velocities
            linear_velocity = self.k_linear * distance
            angular_velocity = self.k_angular * angle_diff

        # Create and publish the Twist message
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        self.publisher.publish(twist)
        self.node.get_logger().info(f'Publishing: linear_x={linear_velocity:.2f}, angular_z={angular_velocity:.2f}')

    def run(self):
        rclpy.spin(self.node)
        rclpy.shutdown()

def main():
    rclpy.init()
    controller = VelocityController()
    controller.run()

if __name__ == '__main__':
    main()

