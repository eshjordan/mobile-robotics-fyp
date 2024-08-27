import struct
from dataclasses import dataclass

ENDIAN_FMT = "<"


@dataclass
class EpuckHeartbeatPacket:
    id: int = 0x0  # byte

    def pack(self):
        return struct.pack(
            ENDIAN_FMT + "B",
            self.id,
        )

    @classmethod
    def unpack(cls, buffer: bytes):
        args = struct.unpack(ENDIAN_FMT + "B", buffer)
        return cls(*args)

    @classmethod
    def calcsize(cls):
        return struct.calcsize(ENDIAN_FMT + "B")


@dataclass
class EpuckKnowledgePacket:
    N: int = 0x0  # byte

    def pack(self):
        return struct.pack(
            ENDIAN_FMT + "B",
            self.N,
        )

    @classmethod
    def unpack(cls, buffer: bytes):
        args = struct.unpack(ENDIAN_FMT + "B", buffer)
        return cls(*args)

    @classmethod
    def calcsize(cls):
        return struct.calcsize(ENDIAN_FMT + "B")
