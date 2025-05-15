import rclpy
from rclpy.node import Node
from lazy_agent_sim_interfaces.msg import (
    EpuckInteraction,
    EpuckKnowledgePacket,
    EpuckKnowledgeRecord,
    Boundary,
    Centroid,
)

# AGENT INTERACTIONS
import agent_interactions.schemes as schemes

scheme_data = {
    "vickery_1d" : {
        "scheme_handler": schemes.Scheme1dVickery
    },
    "modified_vickery_1d": {
        "scheme_handler": schemes.Scheme1dModifiedVickery
    },
    "modified_simple_vickery_1d": {
        "scheme_handler": schemes.Scheme1dModifiedVickerySimple
    },
    "polygons_2d": {
        "scheme_handler": schemes.Scheme2dPolygons
    },
}


"""
launch with:

    ros2 launch agent_interactions repartitioner.launch.py scheme_name:=polygons_2d robot_id:=0

where 'scheme_name' is one of the names in 'scheme_data'
and 'robot_id' is the id of the robot whose knowledge packet will be updated by the interaction

"""

class Repartitioner(Node):

    def __init__(self):
        super().__init__("repartitioner")

        self.declare_parameter("namespace", "~")
        self.declare_parameter("robot_id", 0)
        self.declare_parameter("scheme_name", "vicker_1d")

        self.get_logger().info(f"Namespace: {self.get_parameter('namespace').value}")
        self.get_logger().info(f"Robot ID: {self.get_parameter('robot_id').value}")
        self.get_logger().info(f"Scheme: {self.get_parameter('scheme_name').value}")

        # instantiate the handler
        self.scheme_handler: schemes.InteractionScheme = scheme_data[self.get_parameter('scheme_name').value]["scheme_handler"]()

        self.subscriber = self.create_subscription(
            EpuckInteraction,
            "/repartition/request",
            self.listener_callback,
            10
        ),

        self.response = self.create_publisher(
            EpuckKnowledgePacket,
            "/repartition/response",
            10
        )

    def listener_callback(self, interaction_msg: EpuckInteraction):

        """
        Both agents should receive EXACTLY the same EpuckInteraction msg.

        'this' and 'other' is useful because in some schemes the ordering of the agents matters.

        The below function will pick out if either 'this' or 'other' refers to this robot and then:
         - Computes the repartition for 'this' or 'other' only, respectively.
         - Publishes the new knowledge packet for either 'this' or 'other' robot only, respectively

        """

        this_packet = interaction_msg.this_robot
        other_packet = interaction_msg.other_robot

        if self.get_parameter("robot_id").value not in [this_packet.robot_id, other_packet.robot_id]:
            # id's don't match. ignore packet
            return
        
        # Exchange all records
        # 'this'
        for this_record in this_packet.known_ids:
            try:
                # Find record in other packet
                idx_other = [record.robot_id for record in other_packet.known_ids].index(this_record.robot_id)  # search other packet's known id's
            except ValueError:
                other_packet.known_ids.append(this_record)
        # 'other'
        for other_record in other_packet.known_ids:
            try:
                # Find record in other packet
                idx_this = [record.robot_id for record in this_packet.known_ids].index(other_record.robot_id)  # search other packet's known id's
            except ValueError:
                this_packet.known_ids.append(other_record)
        

        # Get indexes of records

        # Agents always have records of themself
        this_idx_this = [record.robot_id for record in this_packet.known_ids].index(this_packet.robot_id)       # "this" robot's record of "this" robot 
        other_idx_other = [record.robot_id for record in other_packet.known_ids].index(other_packet.robot_id)   # "other" robot's record of "other" robot
        # Knowledge of each other
        this_idx_other = [record.robot_id for record in this_packet.known_ids].index(other_packet.robot_id)     # "this" robot's record of "other" robot
        other_idx_this = [record.robot_id for record in other_packet.known_ids].index(this_packet.robot_id)     # "other" robot's record of "this" robot 

        # Assume each agent has the most up to date info about itself
        # Ensure both agents have updated info about each other
        this_packet.known_ids[this_idx_other] = other_packet.known_ids[other_idx_other]
        other_packet.known_ids[other_idx_this] = this_packet.known_ids[this_idx_this]
        # TODO update info about ALL agent records (probably not needed since they are not needed for this particular interaction)

        self.get_logger().info(f"This: robot_id={this_packet.robot_id},  this_idx_this={this_idx_this},  this_idx_other={this_idx_other}")
        self.get_logger().info(f"Other: robot_id={other_packet.robot_id},  other_idx_other={other_idx_other},  other_idx_this={other_idx_this}")

        this_agent = self.scheme_handler.agent_from_record(this_packet.known_ids[this_idx_this], this_packet.n)
        other_agent = self.scheme_handler.agent_from_record(other_packet.known_ids[other_idx_other], other_packet.n)

        # self.get_logger().info("\n\n")
        # self.get_logger().info("BEFORE")
        # self.get_logger().info("this_agent")
        # self.get_logger().info(f"vertices: {this_agent["vertices"]}")
        # self.get_logger().info("other_agent")
        # self.get_logger().info(f"vertices: {other_agent["vertices"]}")
        # self.get_logger().info(f"th_l: {other_agent["theta_l"]},  th_u: {other_agent["theta_u"]},   eps: {other_agent["epsilon"]}")

        new_this_agent, new_other_agent = self.scheme_handler.interact(this_agent, other_agent)
        new_this_centroid, new_this_boundary, new_this_n = self.scheme_handler.new_centroid_boundary_n(new_this_agent)
        new_other_centroid, new_other_boundary, new_other_n = self.scheme_handler.new_centroid_boundary_n(new_other_agent)

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
            # if this fires something has gone seriously wrong ...
            raise ValueError("Robot is not this or other agent...")


def main():
    rclpy.init()
    repartitioner = Repartitioner()

    TESTING = False

    if not TESTING:
        rclpy.spin(repartitioner)





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

        record0 = EpuckKnowledgeRecord()
        record0.robot_id = 0
        record0.centroid = Centroid()
        record0.boundary = Boundary(x_points = old_bounds_1)
        record0.seq = 0

        record1 = EpuckKnowledgeRecord()
        record1.robot_id = 1
        record1.centroid = Centroid()
        record1.boundary = Boundary(x_points = old_bounds_2)
        record1.seq = 0

        knowledge_packet_0 = EpuckKnowledgePacket()
        knowledge_packet_0.robot_id = 0
        knowledge_packet_0.seq = 0
        knowledge_packet_0.known_ids = [record0, record1]
        # knowledge_packet_0.n = 20
        knowledge_packet_0.n = len(knowledge_packet_0.known_ids)

        knowledge_packet_1 = EpuckKnowledgePacket()
        knowledge_packet_1.robot_id = 1
        knowledge_packet_1.seq = 0
        knowledge_packet_1.known_ids = [record1]    # doesn't know about 0
        # knowledge_packet_1.known_ids = [record0, record1]
        # knowledge_packet_1.n = 20
        knowledge_packet_1.n = len(knowledge_packet_1.known_ids)

        msg = EpuckInteraction()
        msg.this_robot = knowledge_packet_0
        msg.other_robot = knowledge_packet_1

        repartitioner.get_logger().info(f"Publishing: {msg}")
        publisher.publish(msg)

    def timer_callback_2():

        record0 = EpuckKnowledgeRecord()
        record0.robot_id = 0
        record0.centroid = Centroid( x=2, y=2 )
        record0.boundary = Boundary(x_points = [1, 3, 3, 1], y_points = [3, 3, 1, 1])
        record0.seq = 0

        record1 = EpuckKnowledgeRecord()
        record1.robot_id = 1
        record1.centroid = Centroid(x=3, y=3)
        record1.boundary = Boundary(x_points = [2, 4, 4, 2], y_points = [4, 4, 2, 2])
        record1.seq = 0

        knowledge_packet_0 = EpuckKnowledgePacket()
        knowledge_packet_0.robot_id = 0
        knowledge_packet_0.seq = 0
        knowledge_packet_0.known_ids = [record0]    #  doesn't know about 1
        # knowledge_packet_0.known_ids = [record0, record1]
        knowledge_packet_0.n = len(knowledge_packet_0.known_ids)

        knowledge_packet_1 = EpuckKnowledgePacket()
        knowledge_packet_1.robot_id = 1
        knowledge_packet_1.seq = 0
        knowledge_packet_1.known_ids = [record1]    # doesn't know about 0
        # knowledge_packet_1.known_ids = [record0, record1]
        knowledge_packet_1.n = len(knowledge_packet_1.known_ids)

        msg = EpuckInteraction()
        msg.this_robot = knowledge_packet_0
        msg.other_robot = knowledge_packet_1

        repartitioner.get_logger().info(f"Publishing: {msg}")
        publisher.publish(msg)


    timer_period = 1.0

      # seconds
    repartitioner.create_timer(timer_period, timer_callback_1)

    rclpy.spin(repartitioner)
    # # Shutdown
    # waypoint_controller.destroy_node()
    # rclpy.shutdown()

if __name__ == "__main__":
    main()

