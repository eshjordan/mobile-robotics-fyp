import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.time
# from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from lazy_agent_sim_interfaces.msg import EpuckInteraction, EpuckKnowledgePacket, EpuckKnowledgeRecord, Boundary, Centroid
import numpy as np
import typing

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
        self.scheme_handler: schemes.InteractionScheme = scheme_data[self.get_parameter('scheme_name').value]["scheme_handler"]()

        self.subscriber = self.create_subscription(
            EpuckInteraction,
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

        this_packet = interaction_msg.this_robot
        other_packet = interaction_msg.other_robot

        if self.get_parameter("robot_id").value not in [this_packet.robot_id, other_packet.robot_id]:
            # id's don't match. ignore packet
            return
        
        
        this_idx_this = [record.robot_id for record in this_packet.known_ids].index(this_packet.robot_id)
        this_idx_other = [record.robot_id for record in this_packet.known_ids].index(other_packet.robot_id)
        other_idx_other = [record.robot_id for record in other_packet.known_ids].index(other_packet.robot_id)
        other_idx_this = [record.robot_id for record in other_packet.known_ids].index(this_packet.robot_id)

        self.get_logger().info(f"This: robot_id={this_packet.robot_id},  this_idx_this={this_idx_this}")
        self.get_logger().info(f"Other: robot_id={other_packet.robot_id},  other_idx_other={other_idx_other}")

        this_agent = self.scheme_handler.agent_from_record(this_packet.known_ids[this_idx_this], this_packet.n)
        other_agent = self.scheme_handler.agent_from_record(other_packet.known_ids[other_idx_other], other_packet.n)

        self.get_logger().info("\n\n")
        self.get_logger().info("BEFORE")
        self.get_logger().info("this_agent")
        self.get_logger().info(f"vertices: {this_agent["vertices"]}")
        self.get_logger().info("other_agent")
        self.get_logger().info(f"vertices: {other_agent["vertices"]}")
        # self.get_logger().info(f"th_l: {other_agent["theta_l"]},  th_u: {other_agent["theta_u"]},   eps: {other_agent["epsilon"]}")

        new_this_agent, new_other_agent = self.scheme_handler.interact(this_agent, other_agent)
        new_this_centroid, new_this_boundary, new_this_n = self.scheme_handler.new_centroid_boundary_n(new_this_agent)
        new_other_centroid, new_other_boundary, new_other_n = self.scheme_handler.new_centroid_boundary_n(new_other_agent)

        # self.get_logger().info("\n")
        # self.get_logger().info("AFTER")
        # self.get_logger().info("this_agent")
        # self.get_logger().info(f"th_l: {new_this_agent.theta_l},  th_u: {new_this_agent.theta_u}")
        # self.get_logger().info("other_agent")
        # self.get_logger().info(f"th_l: {new_other_agent.theta_l},  th_u: {new_other_agent.theta_u}")

        self.get_logger().info("\n\n")
        self.get_logger().info("BEFORE")
        self.get_logger().info("this_agent")
        self.get_logger().info(f"vertices: {new_this_agent["vertices"]}")
        self.get_logger().info("other_agent")
        self.get_logger().info(f"vertices: {new_other_agent["vertices"]}")

        # Update record and republish
        if self.get_parameter("robot_id").value == this_packet.robot_id:
            this_record_this, this_record_other = this_packet.known_ids[this_idx_this], this_packet.known_ids[this_idx_other]

            this_record_this.centroid, this_record_this.boundary = new_this_centroid, new_this_boundary
            this_record_other.centroid, this_record_other.boundary = new_other_centroid, new_other_boundary
            this_packet.n = new_this_n
            self.response.publish(this_packet)

        elif self.get_parameter("robot_id").value == other_packet.robot_id:
            other_record_other, other_record_this = other_packet.known_ids[other_idx_other], other_packet.known_ids[other_idx_this]

            other_record_other.centroid, other_record_other.boundary = new_other_centroid, new_other_boundary
            other_record_this.centroid, other_record_this.boundary = new_this_centroid, new_this_boundary
            other_packet.n = new_other_n
            self.response.publish(other_packet)

        else:
            raise ValueError("Robot is not this or other agent...")
        

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
        EpuckInteraction,
        "/repartition/request",
        10
    )

    # Create a timer to publish the dummy message periodically
    def timer_callback_1():

        old_bounds_1 = [0, 1]
        old_bounds_2 = [0.5, 3]

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

        knowledge_packet_1 = EpuckKnowledgePacket()
        knowledge_packet_1.robot_id = 0
        knowledge_packet_1.seq = 0
        knowledge_packet_1.known_ids = [record1, record2]
        knowledge_packet_1.n = 20
        # knowledge_packet_1.n = len(knowledge_packet_1.known_ids)

        knowledge_packet_2 = EpuckKnowledgePacket()
        knowledge_packet_2.robot_id = 1
        knowledge_packet_2.seq = 0
        knowledge_packet_2.known_ids = [record1, record2]
        knowledge_packet_2.n = 20
        # knowledge_packet_2.n = len(knowledge_packet_2.known_ids)
        
        msg = EpuckInteraction()
        msg.this_robot = knowledge_packet_1
        msg.other_robot = knowledge_packet_2

        repartitioner.get_logger().info(f"Publishing: {msg}")
        publisher.publish(msg)

    def timer_callback_2():

        record1 = EpuckKnowledgeRecord()
        record1.robot_id = 0
        record1.centroid = Centroid( x=2, y=2 )
        record1.boundary = Boundary(x_points = [1, 3, 3, 1], y_points = [3, 3, 1, 1])
        record1.seq = 0

        record2 = EpuckKnowledgeRecord()
        record2.robot_id = 1
        record2.centroid = Centroid(x=3, y=3)
        record2.boundary = Boundary(x_points = [2, 4, 4, 2], y_points = [4, 4, 2, 2])
        record2.seq = 0

        knowledge_packet_1 = EpuckKnowledgePacket()
        knowledge_packet_1.robot_id = 0
        knowledge_packet_1.seq = 0
        knowledge_packet_1.known_ids = [record1, record2]
        knowledge_packet_1.n = 20

        knowledge_packet_2 = EpuckKnowledgePacket()
        knowledge_packet_2.robot_id = 1
        knowledge_packet_2.seq = 0
        knowledge_packet_2.known_ids = [record1, record2]
        knowledge_packet_2.n = 20
        
        msg = EpuckInteraction()
        msg.this_robot = knowledge_packet_1
        msg.other_robot = knowledge_packet_2

        repartitioner.get_logger().info(f"Publishing: {msg}")
        publisher.publish(msg)


    timer_period = 1.0
    
      # seconds
    repartitioner.create_timer(timer_period, timer_callback_2)

    rclpy.spin(repartitioner)
    # # Shutdown
    # waypoint_controller.destroy_node()
    # rclpy.shutdown()

if __name__ == "__main__":
    main()

