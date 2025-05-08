#!/usr/bin/env python3

import select
import socket
import socketserver
import threading
import logging
import time
from typing import Callable, Iterable, override

from agent_local_comms_server.packets import (
    EpuckHeartbeatPacket,
    EpuckHeartbeatResponsePacket,
    EpuckNeighbourPacket,
    EpuckKnowledgePacket,
    EpuckKnowledgeRecord,
    Centroid,
    Boundary,
)


class BaseKnowledgeServer:
    def __init__(self, robot_model: "RobotCommsModel"):
        self.robot_model = robot_model

    def start(self):
        pass

    def stop(self):
        pass


class BaseKnowledgeClient:
    def __init__(
        self,
        neighbour: EpuckNeighbourPacket,
        running: Callable[[], bool],
        robot_model: "RobotCommsModel",
    ):
        self.neighbour = neighbour
        self.running = running
        self.robot_model = robot_model

    def start(self):
        pass

    def stop(self):
        pass


class BaseRobotCommsModel:
    robot_id: int
    manager_host: str
    manager_port: int
    robot_comms_host: str
    robot_comms_request_port: int
    robot_knowledge_host: str
    robot_knowledge_exchange_port: str

    seq_: int
    known_ids_: dict[int, EpuckKnowledgeRecord]

    def __init__(
        self,
        robot_id: int,
        manager_host: str,
        manager_port: int,
        robot_comms_host: str,
        robot_comms_request_port: int,
        robot_knowledge_host: str,
        robot_knowledge_exchange_port: int,
    ):
        self.robot_id = robot_id
        self.manager_host = manager_host
        self.manager_port = manager_port
        self.robot_comms_host = robot_comms_host
        self.robot_comms_request_port = robot_comms_request_port
        self.robot_knowledge_host = robot_knowledge_host
        self.robot_knowledge_exchange_port = robot_knowledge_exchange_port

        self.seq_ = 0
        self.centroid_ = Centroid()
        self.boundary_ = Boundary([], [], [])
        self.known_ids_ = dict(
            [
                (
                    robot_id,
                    EpuckKnowledgeRecord(
                        robot_id=self.robot_id,
                        centroid=self.centroid_,
                        boundary=self.boundary_,
                        seq=self.GetSeq(),
                    ),
                )
            ]
        )

    def __del__(self):
        pass

    def start(self):
        pass

    def stop(self):
        pass

    def InsertKnownIds(self, new_ids: Iterable[EpuckKnowledgeRecord]) -> int:
        size_before = self.KnownIdsSize()
        for record in new_ids:
            if (
                record.robot_id not in self.known_ids_
                or self.known_ids_[record.robot_id].seq < record.seq
            ):
                self.known_ids_[record.robot_id] = record

        return self.KnownIdsSize() - size_before

    def GetKnownIds(self) -> list[EpuckKnowledgeRecord]:
        return list(self.known_ids_.values())

    def KnownIdsSize(self):
        return len(self.known_ids_)

    def GetSeq(self):
        self.seq_ += 1
        return self.seq_

    def CreateKnowledgePacket(self):
        # Update the sequence number of the internal record for this robot, so it matches the one in the response
        seq = self.GetSeq()
        new_record = [
            EpuckKnowledgeRecord(
                robot_id=self.robot_id,
                centroid=self.centroid_,
                boundary=self.boundary_,
                seq=seq,
            )
        ]
        self.InsertKnownIds(new_record)

        return EpuckKnowledgePacket(
            robot_id=self.robot_id,
            seq=seq,
            N=self.KnownIdsSize(),
            known_ids=self.GetKnownIds(),
        )

    def SetCentroid(self, centroid: Centroid):
        self.centroid_ = Centroid(centroid)

    def SetBoundary(self, boundary: Boundary):
        self.boundary_ = Boundary(
            boundary.x_points.copy(), boundary.y_points.copy(), boundary.z_points.copy()
        )

    def GetCentroid(self) -> Centroid:
        return Centroid(self.centroid_)

    def GetBoundary(self) -> Boundary:
        return Boundary(
            self.boundary_.x_points.copy(),
            self.boundary_.y_points.copy(),
            self.boundary_.z_points.copy(),
        )


class RobotCommsModel(BaseRobotCommsModel):
    def __init__(
        self,
        robot_id: int,
        manager_host: str,
        manager_port: int,
        robot_comms_host: str,
        robot_comms_request_port: int,
        robot_knowledge_host: str,
        robot_knowledge_exchange_port: int,
        KnowledgeServerClass: type[BaseKnowledgeServer],
        KnowledgeClientClass: type[BaseKnowledgeClient],
        logger=logging.getLogger(__name__),
    ):
        super().__init__(
            robot_id,
            manager_host,
            manager_port,
            robot_comms_host,
            robot_comms_request_port,
            robot_knowledge_host,
            robot_knowledge_exchange_port,
        )

        self.KnowledgeServerClass = KnowledgeServerClass
        self.KnowledgeClientClass = KnowledgeClientClass
        self.logger = logger

        self.heartbeat_thread: threading.Thread = None
        self.heartbeat_client: socket.socket = None

        self.knowledge_request_server: socketserver.ThreadingUDPServer = None
        self.knowledge_request_thread: threading.Thread = None

        self.knowledge_server: BaseKnowledgeServer = None
        self.knowledge_clients: dict[int, BaseKnowledgeClient] = None

        self.knowledge_request_server = socketserver.ThreadingUDPServer(
            (self.robot_comms_host, self.robot_comms_request_port),
            self.server_factory(),
        )

    def server_factory(self):
        robot_model = self

        class ReceiveKnowledgeRequestHandler(socketserver.BaseRequestHandler):
            @override
            def handle(self):
                self.request: tuple[bytes, socket.socket]
                data, client = self.request

                host = self.client_address[0]
                port = self.client_address[1]

                request = EpuckKnowledgePacket.unpack(data)

                robot_model.logger.debug(
                    f"Received knowledge request from {host}:{port}"
                )

                knowledge = robot_model.CreateKnowledgePacket()

                client.sendto(knowledge.pack(), self.client_address)

                robot_model.logger.debug(
                    f"Sending requested knowledge to {host}:{port} - {robot_model.GetKnownIds()}"
                )

        return ReceiveKnowledgeRequestHandler

    def __del__(self):
        self.stop()
        super().__del__()

    def start(self):
        self.heartbeat_thread = threading.Thread(
            target=self.exchange_heartbeats)
        self.heartbeat_thread.daemon = True
        self.heartbeat_thread.start()

        self.knowledge_request_thread = threading.Thread(
            target=self.knowledge_request_server.serve_forever
        )
        self.knowledge_request_thread.daemon = True
        self.knowledge_request_thread.start()

    def exchange_heartbeats(self):
        self.logger.info(
            f"Starting heartbeat exchange client on {self.robot_comms_host}"
        )

        self.knowledge_server = self.KnowledgeServerClass(self)
        self.knowledge_server.start()

        self.heartbeat_client = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM)

        self.knowledge_clients = {}

        while True:
            self.logger.debug(
                f"Sending heartbeat to {self.manager_host}:{self.manager_port}"
            )
            self.heartbeat_client.sendto(
                EpuckHeartbeatPacket(
                    robot_id=self.robot_id,
                    robot_comms_host=self.robot_comms_host,
                    robot_comms_request_port=self.robot_comms_request_port,
                    robot_knowledge_host=self.robot_knowledge_host,
                    robot_knowledge_exchange_port=self.robot_knowledge_exchange_port,
                ).pack(),
                (self.manager_host, self.manager_port),
            )

            r_ready, w_ready, x_ready = select.select(
                [self.heartbeat_client], [], [], 0.01
            )
            if len(r_ready) == 0:
                continue

            data, retaddr = self.heartbeat_client.recvfrom(
                EpuckHeartbeatResponsePacket.calcsize()
            )
            response = EpuckHeartbeatResponsePacket.unpack(data)

            for neighbour in response.neighbours:
                self.logger.debug(
                    f"Received neighbour: {neighbour.robot_id} ({neighbour.host}:{neighbour.port}) at distance {neighbour.dist}"
                )

            # Start threads for new neighbours
            new_in_range_neighbours = [
                neighbour
                for neighbour in response.neighbours
                if neighbour.robot_id not in self.knowledge_clients.keys()
                and neighbour.robot_id < self.robot_id
            ]

            for neighbour in new_in_range_neighbours:
                self.logger.info(
                    f"Starting thread for neighbour {neighbour.robot_id} ({neighbour.host}:{neighbour.port})"
                )

                self.knowledge_clients[neighbour.robot_id] = self.KnowledgeClientClass(
                    neighbour,
                    lambda: neighbour.robot_id in self.knowledge_clients.keys(),
                    self,
                )

                self.knowledge_clients[neighbour.robot_id].start()

            # Stop threads for out of range neighbours
            out_of_range_neighbour_ids = [
                neighbour_id
                for neighbour_id in self.knowledge_clients.keys()
                if neighbour_id
                not in [neighbour.robot_id for neighbour in response.neighbours]
            ]

            for neighbour_id in out_of_range_neighbour_ids:
                self.logger.info(
                    f"Stopping thread for neighbour {neighbour_id}")
                knowledge_client = self.knowledge_clients.pop(neighbour_id)
                knowledge_client.stop()

            time.sleep(1)

    def stop(self):
        if self.heartbeat_client is not None:
            self.heartbeat_client.shutdown(socket.SHUT_RDWR)

        if self.knowledge_request_server is not None:
            self.knowledge_request_server.shutdown()

        if self.knowledge_request_thread is not None:
            self.knowledge_request_thread.join()

        if self.knowledge_clients is not None:
            for knowledge_client in self.knowledge_clients.items():
                knowledge_client.stop()

        if self.knowledge_server is not None:
            self.knowledge_server.stop()
