import struct
from dataclasses import dataclass

ENDIAN_FMT = "<"


@dataclass
class EpuckHeartbeatPacket:
    FMT_STR: str = ENDIAN_FMT + "BB"
    ID: int = 0x20

    id: int = 0x20  # byte
    robot_id: int = 0x0  # byte

    def pack(self):
        return struct.pack(EpuckHeartbeatPacket.FMT_STR, self.id, self.robot_id)

    @classmethod
    def unpack(cls, buffer: bytes):
        if buffer[0] != EpuckHeartbeatPacket.ID:
            raise ValueError(
                f"Invalid message id: {buffer[0]}, expected {EpuckHeartbeatPacket.ID}"
            )
        args = struct.unpack(EpuckHeartbeatPacket.FMT_STR, buffer)
        return cls(*args)

    @classmethod
    def calcsize(cls):
        return struct.calcsize(EpuckHeartbeatPacket.FMT_STR)


@dataclass
class EpuckKnowledgePacket:
    FMT_STR: str = ENDIAN_FMT + "BB"
    ID: int = 0x22

    id: int = 0x22  # byte
    N: int = 0x0  # byte

    def pack(self):
        return struct.pack(EpuckKnowledgePacket.FMT_STR, self.id, self.N)

    @classmethod
    def unpack(cls, buffer: bytes):
        if buffer[0] != EpuckKnowledgePacket.ID:
            raise ValueError(
                f"Invalid message id: {buffer[0]}, expected {EpuckKnowledgePacket.ID}"
            )
        args = struct.unpack(EpuckKnowledgePacket.FMT_STR, buffer)
        return cls(*args)

    @classmethod
    def calcsize(cls):
        return struct.calcsize(EpuckKnowledgePacket.FMT_STR)
