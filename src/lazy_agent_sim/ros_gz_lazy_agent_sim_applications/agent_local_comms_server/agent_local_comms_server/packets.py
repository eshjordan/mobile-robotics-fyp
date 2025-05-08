import struct
from dataclasses import dataclass

import rclpy

ENDIAN_FMT = "<"

MAX_ROBOTS = 10
MAX_HOST_LEN = 18
MAX_BOUNDARY_X_POINTS = 2
MAX_BOUNDARY_Y_POINTS = 0
MAX_BOUNDARY_Z_POINTS = 0

ROBOT_ID_TYPE_FMT_STR: str = "H"  # ushort

EPUCK_HEARTBEAT_PACKET_FMT_STR: str = (
    ENDIAN_FMT +
    f"B{ROBOT_ID_TYPE_FMT_STR}{MAX_HOST_LEN+1}sH{MAX_HOST_LEN+1}sH"
)
EPUCK_HEARTBEAT_PACKET_ID: int = 0x20


@dataclass
class EpuckHeartbeatPacket:
    id: int = 0x20  # byte
    robot_id: int = 0x0  # see above
    robot_comms_host: str = ""  # str
    robot_comms_request_port: int = 0  # ushort
    robot_knowledge_host: str = ""  # str
    robot_knowledge_exchange_port: int = 0  # ushort

    def pack(self):
        return struct.pack(
            EPUCK_HEARTBEAT_PACKET_FMT_STR,
            self.id,
            self.robot_id,
            self.robot_comms_host.encode("ascii"),
            self.robot_comms_request_port,
            self.robot_knowledge_host.encode("ascii"),
            self.robot_knowledge_exchange_port,
        )

    @classmethod
    def unpack(cls, buffer: bytes):
        if buffer[0] != EPUCK_HEARTBEAT_PACKET_ID:
            raise ValueError(
                f"Invalid message id: {buffer[0]}, expected {EPUCK_HEARTBEAT_PACKET_ID}"
            )
        args = struct.unpack(EPUCK_HEARTBEAT_PACKET_FMT_STR, buffer)
        obj = cls(*args)
        obj.robot_comms_host = obj.robot_comms_host.decode(
            "ascii").rstrip("\x00")  # type: ignore
        obj.robot_knowledge_host = obj.robot_knowledge_host.decode(
            "ascii").rstrip("\x00")  # type: ignore
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

        start_len = struct.calcsize(EPUCK_HEARTBEAT_RESPONSE_PACKET_FMT_STR)
        id, num_neighbours = struct.unpack(
            EPUCK_HEARTBEAT_RESPONSE_PACKET_FMT_STR, buffer[:start_len]
        )
        neighbours = []
        offset = start_len
        for _ in range(min(num_neighbours, MAX_ROBOTS)):
            neighbour = EpuckNeighbourPacket.unpack(
                buffer[offset: offset + EpuckNeighbourPacket.calcsize()]
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


EPUCK_NEIGHBOUR_PACKET_FMT_STR: str = (
    ENDIAN_FMT + f"{ROBOT_ID_TYPE_FMT_STR}{MAX_HOST_LEN+1}sHf"
)


@dataclass
class EpuckNeighbourPacket:
    robot_id: int = 0  # see above
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
        obj.host = obj.host.decode("ascii").rstrip("\x00")  # type: ignore
        return obj

    @classmethod
    def calcsize(cls):
        return struct.calcsize(EPUCK_NEIGHBOUR_PACKET_FMT_STR)


CENTROID_FMT_STR: str = ENDIAN_FMT + "fff"


@dataclass
class Centroid:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def pack(self):
        return struct.pack(CENTROID_FMT_STR, self.x, self.y, self.z)

    @classmethod
    def unpack(cls, buffer: bytes):
        return cls(*struct.unpack(CENTROID_FMT_STR, buffer))

    @classmethod
    def calcsize(cls):
        return struct.calcsize(CENTROID_FMT_STR)


BOUNDARY_X_POINTS_FMT_STR: str = f'{MAX_BOUNDARY_X_POINTS}f' if MAX_BOUNDARY_X_POINTS > 0 else '1B'
BOUNDARY_Y_POINTS_FMT_STR: str = f'{MAX_BOUNDARY_Y_POINTS}f' if MAX_BOUNDARY_Y_POINTS > 0 else '1B'
BOUNDARY_Z_POINTS_FMT_STR: str = f'{MAX_BOUNDARY_Z_POINTS}f' if MAX_BOUNDARY_Z_POINTS > 0 else '1B'
BOUNDARY_FMT_STR: str = (
    ENDIAN_FMT
    + f'{BOUNDARY_X_POINTS_FMT_STR}{BOUNDARY_Y_POINTS_FMT_STR}{BOUNDARY_Z_POINTS_FMT_STR}'
)


@dataclass
class Boundary:
    x_points: list[float]
    y_points: list[float]
    z_points: list[float]

    def pack(self):
        padded_x_points = self.x_points + [0] * (
            max(MAX_BOUNDARY_X_POINTS, 1) - len(self.x_points)
        )
        padded_y_points = self.y_points + [0] * (
            max(MAX_BOUNDARY_Y_POINTS, 1) - len(self.y_points)
        )
        padded_z_points = self.z_points + [0] * (
            max(MAX_BOUNDARY_Z_POINTS, 1) - len(self.z_points)
        )
        return struct.pack(
            BOUNDARY_FMT_STR,
            *padded_x_points,
            *padded_y_points,
            *padded_z_points,
        )

    @classmethod
    def unpack(cls, buffer: bytes):
        start_len = 0
        next_len = struct.calcsize(BOUNDARY_X_POINTS_FMT_STR)
        x_points = [
            *struct.unpack(BOUNDARY_X_POINTS_FMT_STR, buffer[start_len: start_len + next_len])]
        start_len += next_len
        next_len = struct.calcsize(BOUNDARY_Y_POINTS_FMT_STR)
        y_points = [
            *struct.unpack(BOUNDARY_Y_POINTS_FMT_STR, buffer[start_len: start_len + next_len])]
        start_len += next_len
        next_len = struct.calcsize(BOUNDARY_Z_POINTS_FMT_STR)
        z_points = [
            *struct.unpack(BOUNDARY_Z_POINTS_FMT_STR, buffer[start_len: start_len + next_len])]

        return cls(x_points=x_points, y_points=y_points, z_points=z_points)

    @classmethod
    def calcsize(cls):
        return struct.calcsize(BOUNDARY_FMT_STR)


SEQ_FMT_STR: str = "H"  # ushort
EPUCK_KNOWLEDGE_RECORD_FMT_STR: str = (
    ENDIAN_FMT
    + f"{ROBOT_ID_TYPE_FMT_STR}{CENTROID_FMT_STR[1:]}{BOUNDARY_FMT_STR[1:]}{SEQ_FMT_STR}"
)


@dataclass
class EpuckKnowledgeRecord:
    centroid: Centroid  # Centroid
    boundary: Boundary  # Boundary
    robot_id: int = 0  # see above
    seq: int = 0x0  # ushort

    def pack(self):
        return (
            struct.pack(
                ROBOT_ID_TYPE_FMT_STR,
                self.robot_id,
            )
            + self.centroid.pack()
            + self.boundary.pack()
            + struct.pack(
                SEQ_FMT_STR,
                self.seq,
            )
        )

    @classmethod
    def unpack(cls, buffer: bytes):
        start_len = 0
        next_len = struct.calcsize(ROBOT_ID_TYPE_FMT_STR)
        robot_id = struct.unpack(
            ROBOT_ID_TYPE_FMT_STR, buffer[start_len: start_len + next_len]
        )[0]

        start_len += next_len
        next_len = Centroid.calcsize()
        centroid = Centroid.unpack(buffer[start_len: start_len + next_len])

        start_len += next_len
        next_len = Boundary.calcsize()
        boundary = Boundary.unpack(buffer[start_len: start_len + next_len])

        start_len += next_len
        next_len = struct.calcsize(SEQ_FMT_STR)
        seq = struct.unpack(
            SEQ_FMT_STR, buffer[start_len: start_len + next_len])[0]

        return cls(
            robot_id=robot_id,
            centroid=centroid,
            boundary=boundary,
            seq=seq,
        )

    @classmethod
    def calcsize(cls):
        return struct.calcsize(EPUCK_KNOWLEDGE_RECORD_FMT_STR)

    def __lt__(self, other):
        return self.robot_id < other.robot_id or (
            self.robot_id == other.robot_id and self.seq < other.seq
        )

    def __eq__(self, other):
        return self.robot_id == other.robot_id and self.seq == other.seq

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash((self.robot_id, self.seq))


EPUCK_KNOWLEDGE_PACKET_FMT_STR: str = ENDIAN_FMT + \
    f"B{ROBOT_ID_TYPE_FMT_STR}HB"
EPUCK_KNOWLEDGE_PACKET_ID: int = 0x22


@dataclass
class EpuckKnowledgePacket:
    known_ids: list[EpuckKnowledgeRecord]  # list of EpuckKnowledgeRecord
    id: int = 0x22  # byte
    robot_id: int = 0  # see above
    seq: int = 0  # ushort
    N: int = 0x0  # byte

    def pack(self):
        retval = struct.pack(
            EPUCK_KNOWLEDGE_PACKET_FMT_STR,
            self.id,
            self.robot_id,
            self.seq,
            self.N,
        ) + b"".join([record.pack() for record in self.known_ids]) + b"\x00" * (
            EpuckKnowledgeRecord.calcsize() * (MAX_ROBOTS - len(self.known_ids))
        )
        if len(retval) != self.calcsize():
            rclpy.logging.get_logger("agent_local_comms_server").error(
                f"Packed size {len(retval)} != expected size {self.calcsize()}"
            )
        return retval

    @classmethod
    def unpack(cls, buffer: bytes):
        if buffer[0] != EPUCK_KNOWLEDGE_PACKET_ID:
            raise ValueError(
                f"Invalid message id: {buffer[0]}, expected {EPUCK_KNOWLEDGE_PACKET_ID}"
            )

        start_len = struct.calcsize(EPUCK_KNOWLEDGE_PACKET_FMT_STR)
        id, robot_id, seq, N = struct.unpack(
            EPUCK_KNOWLEDGE_PACKET_FMT_STR, buffer[:start_len]
        )
        known_ids = []
        offset = start_len
        for _ in range(min(N, MAX_ROBOTS)):
            record = EpuckKnowledgeRecord.unpack(
                buffer[offset: offset + EpuckKnowledgeRecord.calcsize()]
            )
            offset += EpuckKnowledgeRecord.calcsize()
            known_ids.append(record)
        return cls(
            id=id,
            robot_id=robot_id,
            seq=seq,
            N=N,
            known_ids=known_ids,
        )

    @classmethod
    def calcsize(cls):
        return (
            struct.calcsize(EPUCK_KNOWLEDGE_PACKET_FMT_STR)
            + EpuckKnowledgeRecord.calcsize() * MAX_ROBOTS
        )
