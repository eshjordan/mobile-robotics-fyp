import struct
from dataclasses import dataclass

ENDIAN_FMT = "<"

MAX_ROBOTS = 10
MAX_HOST_LEN = 16

EPUCK_HEARTBEAT_PACKET_FMT_STR: str = ENDIAN_FMT + f"BB{MAX_HOST_LEN+1}pH"
EPUCK_HEARTBEAT_PACKET_ID: int = 0x20


@dataclass
class EpuckHeartbeatPacket:
    id: int = 0x20  # byte
    robot_id: int = 0x0  # byte
    robot_host: str = ""  # str
    robot_port: int = 0  # ushort

    def pack(self):
        return struct.pack(
            EPUCK_HEARTBEAT_PACKET_FMT_STR,
            self.id,
            self.robot_id,
            self.robot_host.encode("ascii"),
            self.robot_port,
        )

    @classmethod
    def unpack(cls, buffer: bytes):
        if buffer[0] != EPUCK_HEARTBEAT_PACKET_ID:
            raise ValueError(
                f"Invalid message id: {buffer[0]}, expected {EPUCK_HEARTBEAT_PACKET_ID}"
            )
        args = struct.unpack(EPUCK_HEARTBEAT_PACKET_FMT_STR, buffer)
        obj = cls(*args)
        obj.robot_host = obj.robot_host.decode("ascii")
        return obj

    @classmethod
    def calcsize(cls):
        return struct.calcsize(EPUCK_HEARTBEAT_PACKET_FMT_STR)


EPUCK_HEARTBEAT_RESPONSE_PACKET_FMT_STR: str = ENDIAN_FMT + "BB"
EPUCK_HEARTBEAT_RESPONSE_PACKET_ID: int = 0x21


@dataclass
class EpuckHeartbeatResponsePacket:
    neighbours: list["EpuckNeighbourPacket"]  # list of EpuckNeighbourPacket
    id: int = 0x21  # byte
    num_neighbours: int = 0  # byte

    def pack(self):
        return struct.pack(
            EPUCK_HEARTBEAT_RESPONSE_PACKET_FMT_STR,
            self.id,
            self.num_neighbours,
        ) + b"".join([n.pack() for n in self.neighbours])

    @classmethod
    def unpack(cls, buffer: bytes):
        if buffer[0] != EPUCK_HEARTBEAT_RESPONSE_PACKET_ID:
            raise ValueError(
                f"Invalid message id: {buffer[0]}, expected {EPUCK_HEARTBEAT_RESPONSE_PACKET_ID}"
            )

        id, num_neighbours = struct.unpack(
            EPUCK_HEARTBEAT_RESPONSE_PACKET_FMT_STR, buffer[:2]
        )
        neighbours = []
        offset = 2
        for _ in range(min(num_neighbours, MAX_ROBOTS)):
            neighbour = EpuckNeighbourPacket.unpack(
                buffer[offset : offset + EpuckNeighbourPacket.calcsize()]
            )
            offset += EpuckNeighbourPacket.calcsize()
            neighbours.append(neighbour)
        return cls(id=id, num_neighbours=num_neighbours, neighbours=neighbours)

    @classmethod
    def calcsize(cls):
        return (
            struct.calcsize(EPUCK_HEARTBEAT_RESPONSE_PACKET_FMT_STR)
            + MAX_ROBOTS * EpuckNeighbourPacket.calcsize()
        )


EPUCK_NEIGHBOUR_PACKET_FMT_STR: str = ENDIAN_FMT + f"B{MAX_HOST_LEN+1}pHf"


@dataclass
class EpuckNeighbourPacket:
    robot_id: int = 0  # byte
    host: str = ""  # str
    port: int = 0  # ushort
    dist: float = 0.0  # float

    def pack(self):
        return struct.pack(
            EPUCK_NEIGHBOUR_PACKET_FMT_STR,
            self.robot_id,
            self.host.encode("ascii"),
            self.port,
            self.dist,
        )

    @classmethod
    def unpack(cls, buffer: bytes):
        args = struct.unpack(EPUCK_NEIGHBOUR_PACKET_FMT_STR, buffer)
        obj = cls(*args)
        obj.host = obj.host.decode("ascii")
        return obj

    @classmethod
    def calcsize(cls):
        return struct.calcsize(EPUCK_NEIGHBOUR_PACKET_FMT_STR)


EPUCK_KNOWLEDGE_PACKET_FMT_STR: str = ENDIAN_FMT + "BBB"
EPUCK_KNOWLEDGE_PACKET_ID: int = 0x22


@dataclass
class EpuckKnowledgePacket:
    known_ids: list[int]  # list of byte
    id: int = 0x22  # byte
    robot_id: int = 0  # byte
    N: int = 0x0  # byte

    def pack(self):
        return struct.pack(
            EPUCK_KNOWLEDGE_PACKET_FMT_STR,
            self.id,
            self.robot_id,
            self.N,
        ) + bytes(self.known_ids)

    @classmethod
    def unpack(cls, buffer: bytes):
        if buffer[0] != EPUCK_KNOWLEDGE_PACKET_ID:
            raise ValueError(
                f"Invalid message id: {buffer[0]}, expected {EPUCK_KNOWLEDGE_PACKET_ID}"
            )

        id, robot_id, N = struct.unpack(EPUCK_KNOWLEDGE_PACKET_FMT_STR, buffer[:3])
        return cls(id=id, robot_id=robot_id, N=N, known_ids=list(buffer[3 : N + 3]))

    @classmethod
    def calcsize(cls):
        return struct.calcsize(EPUCK_KNOWLEDGE_PACKET_FMT_STR) + MAX_ROBOTS
