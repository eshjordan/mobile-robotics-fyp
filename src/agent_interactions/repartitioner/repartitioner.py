import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.time
# from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from lazy_agent_sim_interfaces.msg import EpuckKnowledgePacket, RepartitionRequest, EpuckKnowledgeRecord, Boundary, Centroid
import numpy as np

# AGENT INTERACTIONS
import agent_interactions.schemes as schemes

"""

Plan:
 - Listens to /.../repartition/scheme1/request
 - Listens to /.../repartition/scheme2/request
 
 ...
 
 - Publishes to /.../repartition/scheme1/response
 - Publishes to /.../repartition/scheme2/response
 
 ...

"""

scheme_data = {
    "vickery_1d" : {
        "scheme_handler": schemes.Scheme1dVickery
    },
    "modified_vickery_1d": {
        "scheme_handler": schemes.Scheme1dModifiedVickery
    },
    "polygons_2d": {
        "scheme_handler": schemes.Scheme2dPolygons

    },
}


class Repartitioner(Node):

    def __init__(self):
        super().__init__("repartitioner")

        self.declare_parameter("namespace", "~")
        self.declare_parameter("robot_id", 0)
        # self.declare_parameter("manager_robot_tf_prefix", "epuck2_robot_")
        # self.declare_parameter("manager_robot_tf_suffix", "")
        # self.declare_parameter("manager_robot_tf_frame", "/base_link")
        self.declare_parameter("scheme_name", "vicker_1d")

        self.get_logger().info(f"Namespace: {self.get_parameter('namespace').value}")
        self.get_logger().info(f"Robot ID: {self.get_parameter('robot_id').value}")
        self.get_logger().info(f"Scheme: {self.get_parameter('scheme_name').value}")

        # instantiate the handler
        self.scheme_handler = scheme_data[self.get_parameter('scheme_name').value]["scheme_handler"]()

        self.subscriber = self.create_subscription(
            RepartitionRequest,
            "/repartition/request",
            # self.get_parameter("namespace").value + "/repartition/request",
            self.listener_callback,
            10
        ),
    
        self.response = self.create_publisher(
            EpuckKnowledgePacket,
            "/repartition/response",
            # self.get_parameter("namespace").value + "/repartition/response",
            10
        )


    def listener_callback(self, interaction_msg):
        # pass

        this_packet = interaction_msg.this_agent
        other_packet = interaction_msg.other_agent

        if this_agent_packet.robot_id != self.get_parameter("robot_id").value:
            return
        
        this_idx = [record.robot_id for record in this_packet.known_ids].index(this_packet.robot_id)
        other_idx = [record.robot_id for record in other_packet.known_ids].index(other_packet.robot_id)

        

        # agent1, agent2 = self.scheme_handler.agents_info_from_request_msg(repartition_request)
        # new_bounds_1, new_bounds_2 = self.scheme_handler.interact(agent1, agent2)
        # # TODO update packet with new boundaries info
        # new_packet = self.scheme_handler.new_knowledge_packet(repartition_request, new_bounds_1, new_bounds_2)
        # self.get_logger().info("\nnew_bounds_1: {}\nnew_bounds_2:{}\n".format(new_bounds_1, new_bounds_2))
        # self.response.publish(new_packet)


def main():
    rclpy.init()
    repartitioner = Repartitioner()
    
    # Create a dummy publisher to send EpuckKnowledgePacket messages
    publisher = repartitioner.create_publisher(
        RepartitionRequest,
        "/epuck_0/epuck_0/repartition/request",
        10
    )

    # Create a timer to publish the dummy message periodically
    def timer_callback():

        old_bounds_1 = [0,np.pi]
        old_bounds_2 = [0.5*np.pi,1.5*np.pi]

        repartitioner.get_logger().info("\nold_bounds_1: {}\nold_bounds_2:{}\n".format(old_bounds_1, old_bounds_2))

        record1 = EpuckKnowledgeRecord()
        record1.robot_id = 0
        record1.centroid = Centroid()
        record1.boundary = Boundary(x_points = old_bounds_1)
        record1.seq = 0

        record2 = EpuckKnowledgeRecord()
        record2.robot_id = 1
        record2.centroid = Centroid()
        record2.boundary = Boundary(x_points = old_bounds_2)
        record2.seq = 0

        knowledge_packet = EpuckKnowledgePacket()
        knowledge_packet.robot_id = 0
        knowledge_packet.seq = 0
        knowledge_packet.known_ids = [record1, record2]
        knowledge_packet.n = len(knowledge_packet.known_ids)
        
        msg = RepartitionRequest()
        msg.knowledge_packet = knowledge_packet
        msg.other_agent_id = 1

        repartitioner.get_logger().info(f"Publishing: {msg}")
        publisher.publish(msg)

    timer_period = 1.0  # seconds
    repartitioner.create_timer(timer_period, timer_callback)

    rclpy.spin(repartitioner)
    # # Shutdown
    # waypoint_controller.destroy_node()
    # rclpy.shutdown()

if __name__ == "__main__":
    main()
