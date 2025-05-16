#!/usr/bin/env python3

import rclpy
import rclpy.time
import rclpy.duration
import rclpy.node
from agent_local_comms_server.packets import (
    MAX_BOUNDARY_X_POINTS,
    MAX_BOUNDARY_Y_POINTS,
    MAX_BOUNDARY_Z_POINTS,
    EpuckKnowledgePacket,
    EpuckKnowledgeRecord,
    Centroid,
    Boundary,
)
from lazy_agent_sim_interfaces.msg import (
    EpuckKnowledgePacket as EpuckKnowledgePacketMsg,
    EpuckKnowledgeRecord as EpuckKnowledgeRecordMsg,
    Boundary as BoundaryMsg,
    Centroid as CentroidMsg,
)


def knowledge_packet_to_msg(
    packet: EpuckKnowledgePacket
) -> EpuckKnowledgePacketMsg:
    return EpuckKnowledgePacketMsg(
        robot_id=packet.robot_id,
        seq=packet.seq,
        n=packet.N,
        known_ids=[
            EpuckKnowledgeRecordMsg(
                robot_id=record.robot_id,
                centroid=CentroidMsg(
                    x=record.centroid.x,
                    y=record.centroid.y,
                    z=record.centroid.z,
                ),
                boundary=BoundaryMsg(
                    x_points=record.boundary.x_points,
                    y_points=record.boundary.y_points,
                    z_points=record.boundary.z_points,
                ),
                seq=record.seq,
            )
            for record in packet.known_ids
        ],
    )


def msg_to_knowledge_packet(
    msg: EpuckKnowledgePacketMsg
) -> EpuckKnowledgePacket:
    return EpuckKnowledgePacket(
        robot_id=msg.robot_id,
        seq=msg.seq,
        N=msg.n,
        known_ids=[
            EpuckKnowledgeRecord(
                robot_id=record.robot_id,
                centroid=Centroid(
                    x=record.centroid.x,
                    y=record.centroid.y,
                    z=record.centroid.z,
                ),
                boundary=Boundary(
                    x_points=list(record.boundary.x_points),
                    y_points=list(record.boundary.y_points),
                    z_points=list(record.boundary.z_points),
                ),
                seq=record.seq,
            )
            for record in msg.known_ids
        ],
    )


def main():
    interactive = False

    robot_id = 0
    centroid = [1.0, 1.0, 0.0]
    boundary_x = [1.0, 1.0]
    boundary_y = []
    boundary_z = []

    if interactive:
        robot_id = int(input("Enter robot ID [0]: ") or 0)

        centroid = input("Enter centroid (x,y,z) [0,0,0]: ") or "0,0,0"
        centroid = list(map(str.strip, centroid.split(",")))
        # Pad to 3 elements
        centroid += ["0"] * (3 - len(centroid))
        centroid = list(map(float, centroid[:3]))

        boundary_x = input(
            f"Enter boundary x_points (max {MAX_BOUNDARY_X_POINTS}) [{','.join(['0'] * min(3, MAX_BOUNDARY_X_POINTS))}{',...' if MAX_BOUNDARY_X_POINTS > 3 else ''}]: "
        ) or ','.join(['0'] * MAX_BOUNDARY_X_POINTS)
        boundary_x = list(map(str.strip, boundary_x.split(",")))
        # Pad to MAX_BOUNDARY_X_POINTS elements
        boundary_x += ["0"] * (MAX_BOUNDARY_X_POINTS - len(boundary_x))
        boundary_x = list(map(float, boundary_x[:MAX_BOUNDARY_X_POINTS]))

        boundary_y = input(
            f"Enter boundary y_points (max {MAX_BOUNDARY_Y_POINTS}) [{','.join(['0'] * min(3, MAX_BOUNDARY_Y_POINTS))}{',...' if MAX_BOUNDARY_Y_POINTS > 3 else ''}]: "
        ) or ','.join(['0'] * MAX_BOUNDARY_Y_POINTS)
        boundary_y = list(map(str.strip, boundary_y.split(",")))
        # Pad to MAX_BOUNDARY_Y_POINTS elements
        boundary_y += ["0"] * (MAX_BOUNDARY_Y_POINTS - len(boundary_y))
        boundary_y = list(map(float, boundary_y[:MAX_BOUNDARY_Y_POINTS]))

        boundary_z = input(
            f"Enter boundary z_points (max {MAX_BOUNDARY_Z_POINTS}) [{','.join(['0'] * min(3, MAX_BOUNDARY_Z_POINTS))}{',...' if MAX_BOUNDARY_Z_POINTS > 3 else ''}]: "
        ) or ','.join(['0'] * MAX_BOUNDARY_Z_POINTS)
        boundary_z = list(map(str.strip, boundary_z.split(",")))
        # Pad to MAX_BOUNDARY_Z_POINTS elements
        boundary_z += ["0"] * (MAX_BOUNDARY_Z_POINTS - len(boundary_z))
        boundary_z = list(map(float, boundary_z[:MAX_BOUNDARY_Z_POINTS]))

    rclpy.init()
    node = rclpy.node.Node("set_knowledge")
    repartition_publisher = node.create_publisher(
        EpuckKnowledgePacketMsg, "/agent_local_comms_server/repartition", 10
    )

    msg = EpuckKnowledgePacketMsg(
        robot_id=robot_id,
        seq=0,
        n=1,
        known_ids=[
            EpuckKnowledgeRecordMsg(
                robot_id=robot_id,
                centroid=CentroidMsg(
                    x=centroid[0],
                    y=centroid[1],
                    z=centroid[2],
                ),
                boundary=BoundaryMsg(
                    x_points=boundary_x,
                    y_points=boundary_y,
                    z_points=boundary_z,
                ),
                seq=0,
            )
        ],
    )

    repartition_publisher.publish(
        msg
    )

    rclpy.spin(node)


if __name__ == "__main__":
    main()
