#!/usr/bin/env python3

from copy import copy
import math
import select
import socket
import struct
from typing import List, override
from agent_local_comms_server.robot_comms_model import BaseRobotCommsModel
import rclpy
import rclpy.time
import rclpy.duration
import rclpy.node
from agent_local_comms_server.packets import (
    MAX_BOUNDARY_X_POINTS,
    MAX_BOUNDARY_Y_POINTS,
    MAX_BOUNDARY_Z_POINTS,
    EPUCK_COMMAND_REQUEST_KNOWLEDGE,
    EPUCK_COMMAND_SET_KNOWLEDGE,
    Boundary,
    Centroid,
    EpuckCommandPacket,
    EpuckHeartbeatPacket,
    EpuckHeartbeatResponsePacket,
    EpuckKnowledgeRecord,
    EpuckNeighbourPacket,
    EpuckKnowledgePacket,
)
from socketserver import BaseRequestHandler, ThreadingUDPServer
import threading
import tf2_ros.buffer
import tf2_ros.transform_listener
from lazy_agent_sim_interfaces.msg import (
    EpuckKnowledgePacket as EpuckKnowledgePacketMsg,
    EpuckKnowledgeRecord as EpuckKnowledgeRecordMsg,
    EpuckInteraction as EpuckInteractionMsg,
    Boundary as BoundaryMsg,
    Centroid as CentroidMsg,
    EpuckRepartitionReset as EpuckRepartitionResetMsg
)


class LocalCommsManager(rclpy.node.Node):
    def __init__(self):
        super().__init__("local_comms_manager")
        self.declare_parameter("server_host", "127.0.0.1")
        self.declare_parameter("server_port", 50000)
        self.declare_parameter("threshold_dist", 0.5)
        self.declare_parameter("robot_tf_prefix", "epuck2_robot_")
        self.declare_parameter("robot_tf_suffix", "")
        self.declare_parameter("robot_tf_frame", "/base_link")
        self.declare_parameter("remap_ids/0", 0)
        self.declare_parameter("remap_ids/1", 1)
        self.declare_parameter("remap_ids/2", 2)
        self.declare_parameter("remap_ids/3", 3)
        self.declare_parameter("robot_ids", "")


        # List of IDs known ahead of time
        self.robot_ids = [int(id) for id in self.get_parameter("robot_ids").value.split(',') if id]  # split by ',' and ignore empty strings
        self.get_logger().info("Robot IDs: " + str(self.robot_ids))

        self.knowledge_request_timer = self.create_timer(
            1.0, self.request_knowledge_tmr_cb, autostart=False
        )
        self.knowledge_request_publisher = self.create_publisher(
            EpuckKnowledgePacketMsg, "~/knowledge", 10
        )

        self.interaction_publisher = self.create_publisher(
            EpuckInteractionMsg, "~/interaction", 10
        )

        self.repartition_subscriber = self.create_subscription(
            EpuckKnowledgePacketMsg,
            "~/repartition",
            self.repartition_callback,
            10,
        )

        self.reset_publisher = self.create_publisher(
            EpuckRepartitionResetMsg,
            "/repartition/reset",
            10
        )

        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(
            self.tf_buffer, self, spin_thread=True
        )

        self.server = None
        self.server_thread = None
        self.known_robots: dict[int, BaseRobotCommsModel] = {}
        self.known_robots_mut = threading.RLock()

        self.current_neighbours: dict[int, set[int]] = {}

        self.start()

    # Destructor
    def __del__(self):
        self.stop()
        super().__del__()

    def start(self):
        if self.server is not None:
            self.get_logger().warning(
                "Server already running, cannot call start again!"
            )
            return

        server_host = self.get_parameter("server_host").value
        server_port = self.get_parameter("server_port").value
        self.get_logger().info(
            f"Starting server on {server_host}:{server_port}")

        try:
            self.server = ThreadingUDPServer(
                (server_host, server_port), self.create_request_handler()
            )
        except OSError as e:
            raise RuntimeError(
                f"Error starting server on {server_host}:{server_port}: {e}"
            )

        self.server_thread = threading.Thread(
            target=self.server.serve_forever, name="heartbeat-listener"
        )
        self.server_thread.daemon = True
        self.server_thread.start()

        self.knowledge_request_timer.reset()


    def stop(self):
        self.knowledge_request_timer.cancel()

        if self.server is None:
            return
        self.server.shutdown()
        self.server.server_close()
        self.server = None

    def robot_topic_name(self, id: int, remap: bool = True) -> str:
        id_str = str(id)

        if remap:
            remappings = self.get_parameters(
                [f"remap_ids/{id}" for id in range(4)])

            for remapping in remappings:
                if remapping.value == id:
                    id_str = remapping.name.split("/")[-1]
                    break

        return (
            f"{self.get_parameter("robot_tf_prefix").value}"
            + f"{id_str}"
            + f"{self.get_parameter("robot_tf_suffix").value}"
        )

    def robot_frame_name(self, id: int, remap: bool = True) -> str:
        return (
            self.robot_topic_name(id, remap)
            + f"{self.get_parameter("robot_tf_frame").value}"
        )

    def create_request_handler(manager: "LocalCommsManager"):
        class Handler(BaseRequestHandler):
            @override
            def handle(self):
                def tf_distance(tf: tf2_ros.TransformStamped) -> float:
                    return math.sqrt(
                        tf.transform.translation.x**2
                        + tf.transform.translation.y**2
                        + tf.transform.translation.z**2
                    )

                threshold_dist = manager.get_parameter("threshold_dist").value

                self.request: tuple[bytes, socket.socket]
                data, client = self.request

                # Unpack heartbeat message from the connected robot
                try:
                    heartbeat = EpuckHeartbeatPacket.unpack(data)
                except (struct.error, ValueError) as e:
                    manager.get_logger().warning(
                        f"Received invalid heartbeat packet: {e}, buffer: {data}"
                    )
                    return

                with manager.known_robots_mut:

                    manager.get_logger().debug(
                        f"manager.known_robots: {manager.known_robots}"
                    )

                    if heartbeat.robot_id not in manager.known_robots:
                        manager.known_robots[heartbeat.robot_id] = BaseRobotCommsModel(
                            heartbeat.robot_id,
                            manager.get_parameter("server_host").value,
                            manager.get_parameter("server_port").value,
                            heartbeat.robot_comms_host,
                            heartbeat.robot_comms_request_port,
                            heartbeat.robot_knowledge_host,
                            heartbeat.robot_knowledge_exchange_port,
                        )

                        manager.reset_publisher.publish(
                            EpuckRepartitionResetMsg(
                                robot_id = heartbeat.robot_id,
                                agent_idx = len(manager.known_robots),
                                num_agents = len(manager.known_robots)
                            )
                        )

                    frame = manager.robot_frame_name(heartbeat.robot_id, False)

                    # Find neighbouring robots
                    known_frames = [
                        (
                            robot.robot_id,
                            robot.robot_knowledge_host,
                            robot.robot_knowledge_exchange_port,
                            manager.robot_frame_name(robot.robot_id, False),
                        )
                        for robot in manager.known_robots.values()
                        if robot.robot_id != heartbeat.robot_id
                    ]

                # Get all transforms to known frames
                transforms = [
                    (
                        id,
                        host,
                        port,
                        manager.tf_buffer.lookup_transform(
                            other_frame, frame, rclpy.time.Time()
                        ),
                    )
                    for id, host, port, other_frame in known_frames
                    if manager.tf_buffer.can_transform(
                        other_frame, frame, rclpy.time.Time()
                    )
                ]

                # Calculate distances to all known robots
                distances = [
                    (id, host, port, tf_distance(tf))
                    for id, host, port, tf in transforms
                ]

                # Filter out all robots that are too far away
                neighbours = [
                    (id, host, port, dist)
                    for id, host, port, dist in distances
                    if dist <= threshold_dist
                ]

                if heartbeat.robot_id not in manager.current_neighbours:
                    manager.current_neighbours[heartbeat.robot_id] = set()

                neighbours_set = set([id for id, _, _, _ in neighbours])
                new_out_of_range = manager.current_neighbours[
                    heartbeat.robot_id
                ].difference(neighbours_set)
                new_in_range = neighbours_set.difference(
                    manager.current_neighbours[heartbeat.robot_id]
                )
                manager.current_neighbours[heartbeat.robot_id] = neighbours_set

                for id in new_out_of_range:
                    manager.get_logger().info(
                        f"Robot {heartbeat.robot_id} going out of range of {id}"
                    )

                for id in new_in_range:
                    manager.get_logger().info(
                        f"Robot {heartbeat.robot_id} coming into range of {id}"
                    )

                # Send response with all neighbour ids and distances
                neighbour_packets = [
                    EpuckNeighbourPacket(
                        robot_id=id,
                        host=host,
                        port=port,
                        dist=dist,
                    )
                    for id, host, port, dist in neighbours
                ]

                # Publish to the interaction topic
                if len(new_in_range) > 0:
                    manager.publish_interactions(
                        heartbeat.robot_id,
                        new_in_range,
                    )

                response = EpuckHeartbeatResponsePacket(
                    num_neighbours=len(neighbour_packets),
                    neighbours=neighbour_packets,
                ).pack()

                manager.get_logger().debug(
                    f"Sending heartbeat response to {self.client_address}"
                )

                # response = [0x21, len(neighbour_ids) + neighbour_id + len(neighbour_host) + neighbour_host + neighbour_port + neightbour_dist + ...]
                client.sendto(response, self.client_address)

                return

        return Handler

    def request_knowledge_tmr_cb(self):
        with self.known_robots_mut:
            known_robots_copy = self.known_robots.copy()

        threads: List[threading.Thread] = []
        for robot in known_robots_copy:
            threads.append(
                threading.Thread(
                    target=self.request_knowledge,
                    args=(robot,),
                )
            )
            threads[-1].start()

        for thread in threads:
            thread.join()

    def request_knowledge(self, robot_id: int):
        self.get_logger().debug(f"Requesting knowledge from {robot_id}")

        with self.known_robots_mut:
            if robot_id not in self.known_robots:
                self.get_logger().warning(
                    f"Robot {robot_id} not found in known robots, cannot request knowledge"
                )
                return None

            robot = self.known_robots[robot_id]

        command = EpuckCommandPacket(
            command=EPUCK_COMMAND_REQUEST_KNOWLEDGE, data=[]).pack()

        self.get_logger().debug("Command packet packed")

        client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        client.sendto(
            command,
            (robot.robot_comms_host, robot.robot_comms_request_port),
        )

        self.get_logger().debug("Command packet sent")

        # Wait for response
        ready_to_read, _, _ = select.select(
            [client], [], [], 1.0
        )

        if not ready_to_read or len(ready_to_read) == 0:
            self.get_logger().warning(
                f"Timeout waiting for knowledge request response from {robot.robot_id}"
            )
            return None

        data, retaddr = client.recvfrom(
            EpuckKnowledgePacket.calcsize()
        )

        self.get_logger().debug("Knowledge packet received")

        if len(data) == 0:
            self.get_logger().warning(
                f"Received empty knowledge request response from {robot.robot_id}"
            )
            return None

        try:
            response = EpuckKnowledgePacket.unpack(data)
        except (struct.error, ValueError) as e:
            self.get_logger().error(
                f"Received invalid knowledge request response from {robot.robot_id}: {e}, buffer: {data}"
            )
            raise RuntimeError(
                f"Received invalid knowledge request response from {robot.robot_id}: {e}, buffer: {data}"
            )

        self.get_logger().debug(
            f"Received knowledge request response from {robot_id} ({robot.robot_id} - {robot.robot_comms_host}:{robot.robot_comms_request_port}): {response}"
        )

        # Remove trailing 'inf's
        for record in response.known_ids:
            record.boundary.x_points = [pt for pt in record.boundary.x_points if abs(pt)!=float('inf')]
            record.boundary.y_points = [pt for pt in record.boundary.y_points if abs(pt)!=float('inf')]
            record.boundary.z_points = [pt for pt in record.boundary.z_points if abs(pt)!=float('inf')]

        self.update_robot_model(robot.robot_id, response)

        msg_response = self.knowledge_packet_to_msg(response)

        self.knowledge_request_publisher.publish(msg_response)

        return response

    def update_robot_model(self, robot_id: int, response: EpuckKnowledgePacket) -> None:
        with self.known_robots_mut:
            if robot_id not in self.known_robots:
                self.get_logger().warning(
                    f"Robot {robot_id} not found in known robots, cannot update model"
                )
                return

            robot = self.known_robots[robot_id]
            robot.seq_ = response.seq
            robot.n_ = response.N
            robot.known_ids_ = {
                record.robot_id: record for record in response.known_ids
            }
            robot_record = robot.known_ids_[robot.robot_id]
            robot.centroid_ = robot_record.centroid
            robot.boundary_ = robot_record.boundary

    def knowledge_packet_to_msg(
        self, packet: EpuckKnowledgePacket
    ) -> EpuckKnowledgePacketMsg:
        return EpuckKnowledgePacketMsg(
            robot_id=packet.robot_id,
            seq=packet.seq,
            n=packet.N,
            num_known_ids=packet.num_known_ids,
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
        self, msg: EpuckKnowledgePacketMsg
    ) -> EpuckKnowledgePacket:
        packet = EpuckKnowledgePacket(
            robot_id=msg.robot_id,
            seq=msg.seq,
            N=msg.n,
            num_known_ids=msg.num_known_ids,
            known_ids=[
                EpuckKnowledgeRecord(
                    robot_id=record.robot_id,
                    centroid=Centroid(
                        x=record.centroid.x,
                        y=record.centroid.y,
                        z=record.centroid.z,
                    ),
                    boundary=Boundary(
                        x_points=list(record.boundary.x_points) if MAX_BOUNDARY_X_POINTS > 0 else [],
                        y_points=list(record.boundary.y_points) if MAX_BOUNDARY_Y_POINTS > 0 else [],
                        z_points=list(record.boundary.z_points) if MAX_BOUNDARY_Z_POINTS > 0 else [],
                    ),
                    seq=record.seq,
                )
                for record in msg.known_ids
            ],
        )
        return packet

    def publish_interactions(self, this_robot_id, new_in_range):
        this_robot_knowledge = self.request_knowledge(
            this_robot_id)

        if not this_robot_knowledge:
            self.get_logger().warning(
                f"Problem requesting knowledge from robot {this_robot_id} when publishing interaction!"
            )
            return

        this_robot_knowledge_msg = self.knowledge_packet_to_msg(
            this_robot_knowledge
        )

        with self.known_robots_mut:
            known_robots_copy = self.known_robots.copy()

        for id in new_in_range:
            other_robot = known_robots_copy[id]

            other_robot_knowledge = self.request_knowledge(
                other_robot.robot_id
            )

            if not other_robot_knowledge:
                self.get_logger().warning(
                    f"Problem requesting knowledge from robot {other_robot.robot_id} when publishing interaction!"
                )
                continue

            other_robot_knowledge_msg = self.knowledge_packet_to_msg(
                other_robot_knowledge
            )

            self.interaction_publisher.publish(
                EpuckInteractionMsg(
                    this_robot=this_robot_knowledge_msg,
                    other_robot=other_robot_knowledge_msg,
                )
            )

    def repartition_callback(
        self, msg: EpuckKnowledgePacketMsg
    ) -> None:
        self.get_logger().debug(
            f"Repartition callback received: {msg}"
        )

        self.get_logger().debug(f"Sending knowledge to {msg.robot_id}")

        with self.known_robots_mut:
            if msg.robot_id not in self.known_robots:
                self.get_logger().warning(
                    f"Robot {msg.robot_id} not found in known robots, cannot set knowledge"
                )
                return None

            robot = self.known_robots[msg.robot_id]

        command = EpuckCommandPacket(
            command=EPUCK_COMMAND_SET_KNOWLEDGE, data=[]).pack()

        self.get_logger().debug("Command packet packed")

        client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        client.sendto(
            command,
            (robot.robot_comms_host, robot.robot_comms_request_port),
        )

        self.get_logger().debug("Command packet sent")

        knowledge = self.msg_to_knowledge_packet(msg)

        data = knowledge.pack()
        self.get_logger().debug("Knowledge packet packed")
        client.sendto(
            data,
            (robot.robot_comms_host, robot.robot_comms_request_port),
        )
        self.get_logger().debug("Knowledge packet sent")


def main():
    rclpy.init()
    local_comms_manager = LocalCommsManager()
    rclpy.spin(local_comms_manager)


if __name__ == "__main__":
    main()
