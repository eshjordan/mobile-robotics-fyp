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
from typing import Callable, override
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
                EpuckHeartbeatPacket(
                    robot_id=self.robot_id,
                    robot_host=self.robot_host,
                    robot_port=self.robot_port,
                ).pack(),
                (self.manager_host, self.manager_port),
            )

            data_to_read = select.select([self.robot_client], [], [], 0.01)[0]
            if not data_to_read:
                continue

            data, retaddr = self.robot_client.recvfrom(
                EpuckHeartbeatResponsePacket.calcsize()
            )
            response = EpuckHeartbeatResponsePacket.unpack(data)

            for neighbour in response.neighbours:
                self.get_logger().debug(
                    f"Received neighbour: {neighbour.robot_id} ({neighbour.host}:{neighbour.port}) at distance {neighbour.dist}"
                )

            # Start threads for new neighbours
            new_in_range_neighbours = [
                neighbour
                for neighbour in response.neighbours
                if neighbour.robot_id not in self.client_threads.keys()
                and neighbour.robot_id < self.robot_id
            ]

            for neighbour in new_in_range_neighbours:
                self.get_logger().info(
                    f"Starting thread for neighbour {neighbour.robot_id} ({neighbour.host}:{neighbour.port})"
                )
                client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.client_threads[neighbour.robot_id] = (
                    client,
                    threading.Thread(
                        target=self.send_knowledge,
                        args=(
                            client,
                            neighbour,
                            lambda: neighbour.robot_id in self.client_threads.keys(),
                        ),
                    ),
                )

                self.client_threads[neighbour.robot_id][1].daemon = True
                self.client_threads[neighbour.robot_id][1].start()

            # Stop threads for out of range neighbours
            out_of_range_neighbour_ids = [
                neighbour_id
                for neighbour_id in self.client_threads.keys()
                if neighbour_id
                not in [neighbour.robot_id for neighbour in response.neighbours]
            ]

            for neighbour_id in out_of_range_neighbour_ids:
                self.get_logger().info(
                    f"Stopping thread for neighbour {neighbour.robot_id} ({neighbour.host}:{neighbour.port})"
                )
                client, thread = self.client_threads.pop(neighbour_id)
                thread.join()

            time.sleep(1)

    def stop(self):
        for _, (client, thread) in self.client_threads.items():
            client.shutdown(socket.SHUT_RDWR)
            thread.join()
        self.robot_server.shutdown()

    def send_knowledge(
        self,
        client: socket.socket,
        neighbour: EpuckNeighbourPacket,
        running: Callable[[], bool],
    ):
        self.get_logger().info(
            f"Starting knowledge connection with {neighbour.robot_id} ({neighbour.host}:{neighbour.port})"
        )
        while running():
            knowledge = EpuckKnowledgePacket(
                robot_id=self.robot_id,
                N=len(self.known_ids),
                known_ids=list(self.known_ids),
            )

            client.sendto(knowledge.pack(), (neighbour.host, neighbour.port))

            self.get_logger().debug(
                f"Sending knowledge to {neighbour.robot_id} ({neighbour.host}:{neighbour.port}): {self.known_ids}"
            )

            data, retaddr = client.recvfrom(EpuckKnowledgePacket.calcsize())
            if len(data) == 0:
                self.get_logger().warning(
                    f"({neighbour.host}:{neighbour.port}) disconnected"
                )
                break

            response = EpuckKnowledgePacket.unpack(data)

            other_known_ids = set(response.known_ids)
            difference = other_known_ids.difference(self.known_ids)
            self.known_ids.update(other_known_ids)

            if len(difference) > 0:
                self.get_logger().info(
                    f"Received new IDs from ({neighbour.host}:{neighbour.port}): {difference}"
                )

            self.get_logger().debug(
                f"Received knowledge from {neighbour.robot_id} ({neighbour.host}:{neighbour.port}): {other_known_ids}"
            )

            time.sleep(1)

    def create_handler(robot_model):
        class ReceiveKnowledgeHandler(socketserver.BaseRequestHandler):
            @override
            def handle(self):
                self.request: tuple[bytes, socket.socket]
                data, client = self.request

                host = self.client_address[0]
                port = self.client_address[1]

                response = EpuckKnowledgePacket.unpack(data)

                other_known_ids = set(response.known_ids)
                difference = other_known_ids.difference(robot_model.known_ids)
                robot_model.known_ids.update(other_known_ids)

                if len(difference) > 0:
                    robot_model.get_logger().info(
                        f"Received new IDs from ({host}:{port}): {difference}"
                    )

                robot_model.get_logger().debug(
                    f"Received knowledge from {response.robot_id} ({host}:{port}): {other_known_ids}"
                )

                knowledge = EpuckKnowledgePacket(
                    robot_id=robot_model.robot_id,
                    N=len(robot_model.known_ids),
                    known_ids=list(robot_model.known_ids),
                )

                client.sendto(knowledge.pack(), self.client_address)

                robot_model.get_logger().debug(
                    f"Sending knowledge to {response.robot_id} ({host}:{port}): {robot_model.known_ids}"
                )

                return

        return ReceiveKnowledgeHandler


def main():
    rclpy.init()
    robot_model = RobotCommsModel()
    rclpy.spin(robot_model)


if __name__ == "__main__":
    main()
