#!/usr/bin/env python3

import select
from agent_local_comms_server.packets import EpuckKnowledgePacket
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
from .packets import (
    EpuckHeartbeatPacket,
    EpuckHeartbeatResponsePacket,
    EpuckNeighbourPacket,
)

ENDIAN_FMT = "<"


class RobotCommsModel(rclpy.node.Node):
    def __init__(self):
        super().__init__("robot_comms_model")

        self.robot_id = (
            self.declare_parameter("robot_id", 0).get_parameter_value().integer_value
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
        self.client_threads: dict[int, tuple[socket.socket, threading.Thread]] = None

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
        self.get_logger().info(
            f"Starting robot comms model on {self.robot_host}:{self.robot_port}"
        )
        self.robot_server = socketserver.ThreadingUDPServer(
            (self.robot_host, self.robot_port), self.create_handler()
        )

        self.server_thread = threading.Thread(target=self.robot_server.serve_forever)
        self.server_thread.daemon = True
        self.server_thread.start()

        self.robot_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.client_threads = {}

        while True:
            self.get_logger().debug(
                f"Sending heartbeat to {self.manager_host}:{self.manager_port}"
            )
            self.robot_client.sendto(
                EpuckHeartbeatPacket(robot_id=self.robot_id).pack(),
                (self.manager_host, self.manager_port),
            )

            data_to_read = select.select([self.robot_client], [], [], 0.01)[0]
            if not data_to_read:
                continue

            data, retaddr = self.robot_client.recvfrom(
                EpuckHeartbeatResponsePacket.calcsize()
            )
            response = EpuckHeartbeatResponsePacket.unpack(data)

            neighbours = []
            for neighbour in response.neighbours:
                neighbours.append(neighbour)
                self.get_logger().info(
                    f"Received neighbour: {neighbour.robot_id} ({neighbour.host}:{neighbour.port}) at distance {neighbour.dist}"
                )

                if (
                    neighbour.robot_id < self.robot_id
                    and neighbour.robot_id not in self.client_threads
                ):
                    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    self.client_threads[neighbour.robot_id] = (
                        client,
                        threading.Thread(
                            target=self.send_knowledge,
                            args=(client, neighbour),
                        ),
                    )

                    self.client_threads[neighbour.robot_id][1].daemon = True
                    self.client_threads[neighbour.robot_id][1].start()

            time.sleep(1)

    def stop(self):
        for _, (client, _) in self.client_threads.items():
            client.shutdown()
        self.robot_server.shutdown()

    def send_knowledge(
        self,
        client: socket.socket,
        neighbour: EpuckNeighbourPacket,
    ):
        self.get_logger().info("Sending knowledge packets")
        while True:
            knowledge = EpuckKnowledgePacket(
                robot_id=self.robot_id,
                N=len(self.known_ids),
                known_ids=list(self.known_ids),
            )
            client.sendto(knowledge.pack(), (neighbour.host, neighbour.port))

            data, retaddr = self.robot_client.recvfrom(EpuckKnowledgePacket.calcsize())

            response = EpuckKnowledgePacket.unpack(data)
            other_known_ids = set(response.known_ids)
            self.known_ids.update(other_known_ids)

            self.get_logger().info(
                f"Received knowledge from {neighbour.robot_id} ({neighbour.host}:{neighbour.port}): {other_known_ids}"
            )
            time.sleep(1)

    def create_handler(robot_model):
        class ReceiveKnowledgeHandler(socketserver.BaseRequestHandler):
            @override
            def handle(self):
                robot_model.get_logger().info("Received knowledge packet")

                self.request: tuple[bytes, socket.socket]
                data, client = self.request

                # Parse packet
                response = EpuckKnowledgePacket.unpack(data)
                other_known_ids = set(response.known_ids)
                robot_model.known_ids.update(other_known_ids)

                robot_model.get_logger().info(
                    f"Received knowledge from {self.client_address}: {other_known_ids}"
                )

                knowledge = EpuckKnowledgePacket(
                    robot_id=robot_model.robot_id,
                    N=len(robot_model.known_ids),
                    known_ids=list(robot_model.known_ids),
                )

                client.sendto(knowledge.pack(), self.client_address)

                return

        return ReceiveKnowledgeHandler


def main():
    rclpy.init()
    robot_model = RobotCommsModel()
    rclpy.spin(robot_model)


if __name__ == "__main__":
    main()
