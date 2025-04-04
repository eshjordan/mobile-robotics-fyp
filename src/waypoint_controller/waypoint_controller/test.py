import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class WaypointListener(Node):
    def __init__(self):
        super().__init__('waypoint_listener')  # Node name
        self.subscription = self.create_subscription(
            Twist,
            'mobile_base/cmd_vel',  # The topic we are subscribing to
            self.listener_callback,  # Callback function when a message is received
            10  # QoS (Quality of Service), 10 means the queue size
        )

    def listener_callback(self, msg):
        # This function is called when a new Twist message is received
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Log the values received
        self.get_logger().info(f'Received twist message: linear.x = {linear_x}, angular.z = {angular_z}')
        
def main():
    rclpy.init()
    print("h")
    waypoint_listener = WaypointListener()

    # Spin to keep the listener alive and processing incoming messages
    rclpy.spin(waypoint_listener)

    # Shutdown (though not really necessary for this simple example)
    waypoint_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
