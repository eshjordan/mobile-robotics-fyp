#!/usr/bin/env python3

import math
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
)
from socketserver import BaseRequestHandler, ThreadingUDPServer
import threading
import tf2_ros.buffer
import tf2_ros.transform_listener


ENDIAN_FMT = "<"
MAX_ROBOTS = 10
MAX_HOST_LEN = 16


class LocalCommsManager(rclpy.node.Node):
    def __init__(self):
        super().__init__("local_comms_manager")
        self.declare_parameter("server_host", "127.0.0.1")
        self.declare_parameter("server_port", 50000)
        self.declare_parameter("threshold_dist", 0.5)
        self.declare_parameter("robot_tf_prefix", "epuck2_robot_")
        self.declare_parameter("robot_tf_suffix", "")
        self.declare_parameter("robot_tf_frame", "base_link")

        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(
            self.tf_buffer, self, spin_thread=True
        )

        self.server = None
        self.server_thread = None
        self.known_robots: set[tuple[int, str, int]] = set()

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
        self.get_logger().info(f"Starting server on {server_host}:{server_port}")
        self.server = ThreadingUDPServer(
            (server_host, server_port), self.create_request_handler()
        )

        self.server_thread = threading.Thread(
            target=self.server.serve_forever, name="heartbeat-listener"
        )
        self.server_thread.daemon = True
        self.server_thread.start()

    def stop(self):
        if self.server is None:
            return
        self.server.shutdown()
        self.server.server_close()
        self.server = None

    def frame_name(self, id: int) -> str:
        return (
            f"{self.get_parameter("robot_tf_prefix").value}"
            + f"{id}"
            + f"{self.get_parameter("robot_tf_suffix").value}/"
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

                manager.get_logger().debug(
                    f"manager.known_robots: {manager.known_robots}"
                )

                if (
                    heartbeat.robot_id,
                    heartbeat.robot_host,
                    heartbeat.robot_port,
                ) not in manager.known_robots:
                    manager.known_robots.add(
                        (
                            heartbeat.robot_id,
                            heartbeat.robot_host,
                            heartbeat.robot_port,
                        )
                    )

                frame = manager.frame_name(heartbeat.robot_id)

                # Find neighbouring robots
                known_frames = [
                    (id, host, port, manager.frame_name(id))
                    for id, host, port in manager.known_robots
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

                # Send response with all neighbour ids and distances
                neighbour_packets = [
                    EpuckNeighbourPacket(robot_id=id, host=host, port=port, dist=dist)
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


def main():
    rclpy.init()
    local_comms_manager = LocalCommsManager()
    rclpy.spin(local_comms_manager)


if __name__ == "__main__":
    main()
