#!/usr/bin/env python3

import math
import select
import socket
import struct
from typing import override
import rclpy
import rclpy.time
import rclpy.duration
import rclpy.node
from agent_local_comms_server.packets import (
    EpuckHeartbeatPacket,
    EpuckHeartbeatResponsePacket,
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
    Boundary as BoundaryMsg,
    Centroid as CentroidMsg,
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

        self.knowledge_request_client = None
        self.knowledge_request_timer = self.create_timer(
            1.0, self.request_knowledge, autostart=False
        )
        self.knowledge_request_publisher = self.create_publisher(
            EpuckKnowledgePacketMsg, "~/knowledge", 10
        )

        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(
            self.tf_buffer, self, spin_thread=True
        )

        self.server = None
        self.server_thread = None
        self.known_robots: set[tuple[int, str, int]] = set()
        self.known_robots_mut = threading.Lock()

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
        self.server = ThreadingUDPServer(
            (server_host, server_port), self.create_request_handler()
        )

        self.server_thread = threading.Thread(
            target=self.server.serve_forever, name="heartbeat-listener"
        )
        self.server_thread.daemon = True
        self.server_thread.start()

        self.knowledge_request_client = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM)
        self.knowledge_request_timer.reset()

    def stop(self):
        self.knowledge_request_timer.cancel()
        self.knowledge_request_client = None

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

                    if (
                        heartbeat.robot_id,
                        heartbeat.robot_comms_host,
                        heartbeat.robot_comms_request_port,
                        heartbeat.robot_knowledge_host,
                        heartbeat.robot_knowledge_exchange_port,
                    ) not in manager.known_robots:
                        manager.known_robots.add(
                            (
                                heartbeat.robot_id,
                                heartbeat.robot_comms_host,
                                heartbeat.robot_comms_request_port,
                                heartbeat.robot_knowledge_host,
                                heartbeat.robot_knowledge_exchange_port,
                            )
                        )

                    frame = manager.robot_frame_name(heartbeat.robot_id, False)

                    # Find neighbouring robots
                    known_frames = [
                        (
                            id,
                            knowledge_host,
                            knowledge_exchange_port,
                            manager.robot_frame_name(id, False),
                        )
                        for id, _, _, knowledge_host, knowledge_exchange_port in manager.known_robots
                        if id != heartbeat.robot_id
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

                if heartbeat.robot_id not in manager.current_neighbours.keys():
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
                        robot_id=id, host=host, port=port, dist=dist)
                    for id, host, port, dist in neighbours
                ]

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

    def request_knowledge(self):
        with self.known_robots_mut:
            known_robots_copy = self.known_robots.copy()

        for (
            robot_id,
            robot_comms_host,
            robot_comms_request_port,
            _,
            _,
        ) in known_robots_copy:
            self.get_logger().debug(f"Requesting knowledge from {robot_id}")

            request = EpuckKnowledgePacket(
                robot_id=robot_id,
                seq=0,
                N=0,
                known_ids=[],
            ).pack()

            self.get_logger().debug('Knowledge packet packed')

            self.knowledge_request_client.sendto(
                request,
                (robot_comms_host, robot_comms_request_port),
            )

            self.get_logger().debug('Knowledge packet sent')

            # Wait for response
            ready_to_read, _, _ = select.select(
                [self.knowledge_request_client], [], [], 1.0
            )

            if not ready_to_read or len(ready_to_read) == 0:
                self.get_logger().warning(
                    f"Timeout waiting for knowledge request response from {robot_id}"
                )
                continue

            data, retaddr = self.knowledge_request_client.recvfrom(
                EpuckKnowledgePacket.calcsize()
            )

            self.get_logger().debug('Knowledge packet received')

            if len(data) == 0:
                self.get_logger().warning(
                    f"Received empty knowledge request response from {robot_id}"
                )
                continue

            response = EpuckKnowledgePacket.unpack(data)
            self.get_logger().debug(
                f"Received knowledge request response from {robot_id}: {response}"
            )

            msg_response = EpuckKnowledgePacketMsg(
                robot_id=response.robot_id,
                seq=response.seq,
                N=response.N,
                known_ids=[
                    EpuckKnowledgeRecordMsg(
                        robot_id=known_id.robot_id,
                        centroid=CentroidMsg(
                            x=known_id.centroid.x, y=known_id.centroid.y, z=known_id.centroid.z),
                        boundary=BoundaryMsg(x_points=known_id.boundary.x_points,
                                             y_points=known_id.boundary.y_points, z_points=known_id.boundary.z_points),
                        seq=known_id.seq
                    )
                    for known_id in response.known_ids
                ]
            )

            self.knowledge_request_publisher.publish(msg_response)


def main():
    rclpy.init()
    local_comms_manager = LocalCommsManager()
    rclpy.spin(local_comms_manager)


if __name__ == "__main__":
    main()
