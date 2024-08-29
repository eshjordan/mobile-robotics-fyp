#!/usr/bin/env python3

import select
import rclpy
import rclpy.node
import argparse
import socket
import socketserver
import threading

# Now import ThreadPoolExecutor
from concurrent.futures import ThreadPoolExecutor

import struct
import time
from typing import override
from .packets import EpuckHeartbeatPacket

ENDIAN_FMT = "<"

class RobotCommsModel(rclpy.node.Node):
    def __init__(self):
        super().__init__("robot_comms_model")

        self.robot_id = (
            self.declare_parameter("robot_id", 1).get_parameter_value().integer_value
        )
        self.manager_host = (
            self.declare_parameter("manager_host", "127.0.0.1")
            .get_parameter_value()
            .string_value
        )
        self.manager_port = (
            self.declare_parameter("manager_port", 50000)
            .get_parameter_value()
            .integer_value
        )
        self.robot_host = (
            self.declare_parameter("robot_host", "127.0.0.1")
            .get_parameter_value()
            .string_value
        )
        self.robot_port = (
            self.declare_parameter("robot_port", 50001)
            .get_parameter_value()
            .integer_value
        )

        self.known_ids = set([self.robot_id])
        self.robot_server: socketserver.ThreadingUDPServer = None
        self.server_thread: threading.Thread = None
        self.robot_client: socket.socket = None
        self.client_threads: tuple[socket.socket, threading.Thread] = None

        self.start_thread = None
        self.start()

    def __del__(self):
        self.stop()
        super().__del__()

    def start(self):
        self.start_thread = threading.Thread(target=self._start)
        self.start_thread.daemon = True
        self.start_thread.start()

    def _start(self):
        self.robot_server = socketserver.ThreadingUDPServer(
            (self.robot_host, self.robot_port), self.create_handler()
        )

        self.server_thread = threading.Thread(target=self.robot_server.serve_forever)
        self.server_thread.daemon = True
        self.server_thread.start()

        self.robot_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.robot_client.setblocking(False)

        self.client_threads = []

        while True:
            self.get_logger().info(
                f"Sending heartbeat to {self.manager_host}:{self.manager_port}"
            )
            self.robot_client.sendto(
                EpuckHeartbeatPacket(robot_id=self.robot_id).pack(),
                (self.manager_host, self.manager_port),
            )

            data_to_read = select.select([self.robot_client], [], [], 0.01)[0]
            if not data_to_read:
                time.sleep(1)
                continue

            header = self.robot_client.recvfrom(2)
            assert header[0] == 0x21
            num_neighbours = header[1]

            self.get_logger().info("Received heartbeat response")

            neighbours = []
            for _ in range(num_neighbours):
                neighbour_id: int = struct.unpack(ENDIAN_FMT + "B", self.robot_client.recv(1))[0]
                neighbour_host_len = struct.unpack(ENDIAN_FMT + "B", self.robot_client.recv(1))[0]
                neighbour_host = self.robot_client.recv(neighbour_host_len).decode(
                    "ascii"
                )
                neighbour_port: int = struct.unpack(ENDIAN_FMT + "H", self.robot_client.recv(2))[0]
                neighbour_dist: float = struct.unpack(ENDIAN_FMT + "f", self.robot_client.recv(4))[0]

                neighbour_info = (
                    neighbour_id,
                    neighbour_host,
                    neighbour_port,
                    neighbour_dist,
                )
                neighbours.append(neighbour_info)
                self.get_logger().info(
                    f"Received neighbour: {neighbour_id} ({neighbour_host}:{neighbour_port}) at distance {neighbour_dist}"
                )

                if (
                    neighbour_id < self.robot_id
                    and self.client_threads[neighbour_id] is None
                ):
                    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    self.client_threads[neighbour_id] = (
                        client,
                        threading.Thread(
                            target=self.send_knowledge,
                            args=client + neighbour_info,
                        ),
                    )

                    self.client_threads[neighbour_id][1].daemon = True
                    self.client_threads[neighbour_id][1].start()

            time.sleep(1)

    def stop(self):
        for client, _ in self.client_threads:
            client.shutdown()
        self.robot_server.shutdown()

    def send_knowledge(
        self,
        client: socket.socket,
        neighbour_id: int,
        neighbour_host: str,
        neighbour_port: int,
        neighbour_dist: float,
    ):
        while True:
            client.sendto(
                struct.pack(ENDIAN_FMT + "B", self.robot_id)
                + struct.pack(ENDIAN_FMT + "B", len(self.known_ids))
                + bytes(self.known_ids),
                (neighbour_host, neighbour_port),
            )

            other_robot_id = struct.unpack(ENDIAN_FMT + "B", self.robot_client.recv(1))[0]
            num_other_known_ids = struct.unpack(ENDIAN_FMT + "B", self.robot_client.recv(1))[0]
            other_known_ids = set(self.robot_client.recv(num_other_known_ids))

            self.known_ids.update(other_known_ids)
            self.get_logger().info(
                f"Received knowledge from {neighbour_id} ({neighbour_host}:{neighbour_port}): {other_known_ids}"
            )
            time.sleep(1)

    def create_handler(robot_model):
        class ReceiveKnowledgeHandler(socketserver.BaseRequestHandler):
            @override
            def handle(self):
                self.request: tuple[bytes, socket.socket]
                data, client = self.request

                # Parse packet
                other_robot_id = struct.unpack(ENDIAN_FMT + "B", data[0])[0]
                num_other_known_ids = struct.unpack(ENDIAN_FMT + "B", data[1])[0]
                other_known_ids = set(data[2:])

                robot_model.known_ids.update(other_known_ids)
                robot_model.get_logger().info(
                    f"Received knowledge from {self.client_address}: {other_known_ids}"
                )

                client.sendto(
                    struct.pack(ENDIAN_FMT + "B", robot_model.robot_id)
                    + struct.pack(ENDIAN_FMT + "B", len(robot_model.known_ids))
                    + bytes(robot_model.known_ids),
                    self.client_address,
                )

                return

        return ReceiveKnowledgeHandler


def main():
    rclpy.init()
    robot_model = RobotCommsModel()
    rclpy.spin(robot_model)


if __name__ == "__main__":
    main()
