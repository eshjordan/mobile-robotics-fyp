import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist
from math import pi

from lazy_agent_sim_interfaces.msg import (
    EpuckInteraction,
    EpuckKnowledgePacket,
    EpuckKnowledgeRecord,
    Boundary,
    Centroid,
    EpuckRepartitionReset,
)


# specifically for printing bounds / centroid info
def print_bounds(msg: EpuckKnowledgePacket):

    this_idx_this = [record.robot_id for record in msg.known_ids].index(msg.robot_id)       # "this" robot's record of "this" robot 
    boundary: Boundary = msg.known_ids[this_idx_this].boundary
    centroid: Centroid = msg.known_ids[this_idx_this].centroid
    scheme_type = "1d" if len(boundary.x_points) == 2 else "2d"

    if scheme_type == "1d":
        lower_bound = boundary.x_points[0]
        upper_bound = boundary.x_points[1]
        return "ctr: {:5.2f} π   lo: {:5.2f} π   hi: {:5.2f} π".format(centroid.x/pi, lower_bound/pi, upper_bound/pi)
    
    elif scheme_type == "2d":

        vertices = [
            (x,y) for x,y in zip(boundary.x_points, boundary.y_points)
        ]

        return "verts: {}".format(', '.join([str((round(x,2), round(y,2))) for x,y in vertices]))





# Define which robot id's you want to monitor
robot_ids = [5785, 5653, 5731, 5831]

# Define which attributes you want to monitor which differ by topic name id
attributes = [
    {
        'topic' : 'pose',
        'name'  : 'Pose',
        'func' : lambda msg: "x: {:6.3f}  y: {:6.3f}".format(msg.pose.position.x, msg.pose.position.y),
        'msg_type' : PoseStamped
    },
    {
        'topic' : 'mobile_base/cmd_vel',
        'name' : 'Cmd_vel',
        'func' : lambda msg: "fwd: {:6.3f}  rot: {:6.3f}".format(msg.linear.x, msg.angular.z),
        'msg_type' : Twist
    }
]

# Define which attributes you want to monitor which differ by msg 'robot_id'
id_attributes = [
    {
        'topic': '/agent_local_comms_server/repartition',
        'name' : 'bounds',
        'func' : print_bounds,
        'msg_type' : EpuckKnowledgePacket
    }
]




class MyDashboard(Node):

    def __init__(self):

        super().__init__('mydashboard')

        self.subscribers = []

        for i,robot_id in enumerate(robot_ids):

            for attr in attributes:

                self.subscribers.append(

                    self.create_subscription(
                        attr['msg_type'],
                        f"/epuck2_robot_{robot_id}/{attr['topic']}",
                        # callback,
                        lambda msg, idx=i, at=attr: self.callback(idx, at, msg),
                        qos_profile=QoSProfile(
                            reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE,
                            history=HistoryPolicy.KEEP_LAST,
                            depth=10,
                        ),
                    )
                )
            

        for attr in id_attributes:

            self.subscribers.append(

                self.create_subscription(
                    attr['msg_type'],
                    f"{attr['topic']}",
                    # callback,
                    lambda msg, at=attr: self.id_callback(at, msg),
                    qos_profile=QoSProfile(
                        reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE,
                        history=HistoryPolicy.KEEP_LAST,
                        depth=10,
                    ),
                )
            )


        self.data = [{} for _ in range(len(robot_ids))]

        self.create_timer(0.05, self.update_dashboard)



    def callback(self, idx, attr, msg):
        self.data[idx][attr['name']] = attr['func'](msg)



    def id_callback(self, attr, msg):
        idx = robot_ids.index(msg.robot_id)
        self.data[idx][attr['name']] = attr['func'](msg)



    def update_dashboard(self):
        for idx, robot_id in enumerate(robot_ids):
            s = f"{robot_id}:   " + "  |  ".join([f"{key}: {self.data[idx][key]}" for key in sorted(self.data[idx].keys())])
            self.get_logger().info(s)
        self.get_logger().info("")




def main():
    rclpy.init()
    dashboard = MyDashboard()
    rclpy.spin(dashboard)

if __name__ == "__main__":
    main()