"""Serial and feedback fixtures for USB2CAN protocol regressions."""
import struct
from lingzu import protocol as p


class Serial:
    def __init__(self, incoming=b'', partial_write=None):
        self.incoming = bytearray(incoming)
        self.output = bytearray()
        self.partial_write, self.closed = partial_write, False

    @property
    def in_waiting(self):
        return len(self.incoming)

    def read(self, length):
        data = bytes(self.incoming[:length])
        del self.incoming[:length]
        return data

    def write(self, data):
        count = min(len(data), self.partial_write or len(data))
        self.output.extend(data[:count])
        return count

    def close(self):
        self.closed = True


def feedback(mid=1, host=0xFD, fault=0, state=2, data=None):
    return p.Frame((2 << 24) | (state << 22) | (fault << 16) | (mid << 8) | host,
                   struct.pack('>4H', 40000, 32767, 32767, 300) if data is None else data)
