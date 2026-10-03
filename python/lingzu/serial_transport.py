"""Classic USB2CAN serial transport with explicit wire profiles and channels."""
import math
import os
import struct
import threading
import time
from .protocol import Frame

PROFILES = ('soulde_usb2can_v25', 'at_shifted')
PACKET_LAYOUTS = ('fixed8', 'compact')


def require_profile(profile):
    if profile not in PROFILES:
        raise ValueError('Select adapter_profile from ' + ', '.join(PROFILES))


def require_layout(layout):
    if layout not in PACKET_LAYOUTS:
        raise ValueError('Select packet_layout="fixed8" or "compact"')


def crc8(data):
    """Manual v2.5 table: seed FF, reflected polynomial 8C, xor-out zero."""
    value = 0xFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x8C if value & 1 else 0)
    return value


def encode_packet(frame, adapter_profile, *, can_channel=None, packet_layout='fixed8'):
    require_profile(adapter_profile)
    require_layout(packet_layout)
    channel = frame.channel if can_channel is None else can_channel
    frame = Frame(frame.arbitration_id, bytes(frame.data), frame.is_extended_id, channel)
    if adapter_profile == 'at_shifted':
        address = (frame.arbitration_id << 3) | (4 if frame.is_extended_id else 0)
        return b'AT' + struct.pack('>I', address) + bytes([len(frame.data)]) + frame.data + b'\r\n'
    packet = bytes([0xA8, channel]) + struct.pack('<I', frame.arbitration_id)
    packet += bytes([int(frame.is_extended_id), frame.dlc])
    packet += frame.data.ljust(8, b'\0') if packet_layout == 'fixed8' else frame.data
    return packet + bytes([crc8(packet)])


class PacketParser:
    def __init__(self, adapter_profile, *, packet_layout='fixed8'):
        require_profile(adapter_profile)
        require_layout(packet_layout)
        self.profile, self.layout = adapter_profile, packet_layout
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        marker = b'AT' if self.profile == 'at_shifted' else b'\xA9'
        header = 7 if self.profile == 'at_shifted' else 8
        while True:
            start = self.buffer.find(marker)
            if start < 0:
                self.buffer[:] = self.buffer[-1:] if marker == b'AT' and self.buffer.endswith(b'A') else b''
                break
            del self.buffer[:start]
            if len(self.buffer) < header:
                break
            length = self.buffer[header - 1]
            if length > 8:
                del self.buffer[0]
                continue
            size = 9 + length if self.profile == 'at_shifted' or self.layout == 'compact' else 17
            if self.profile != 'at_shifted' and (self.buffer[1] not in (1, 2) or self.buffer[6] > 1):
                del self.buffer[0]
                continue
            if len(self.buffer) < size:
                break
            if self.profile == 'at_shifted':
                if self.buffer[size - 2:size] != b'\r\n':
                    del self.buffer[0]
                    continue
                address = struct.unpack('>I', self.buffer[2:6])[0]
                if address & 3:
                    del self.buffer[0]
                    continue
                arguments = (address >> 3, bytes(self.buffer[7:7 + length]), bool(address & 4), 1)
            else:
                if crc8(self.buffer[:size - 1]) != self.buffer[size - 1]:
                    del self.buffer[0]
                    continue
                address = struct.unpack('<I', self.buffer[2:6])[0]
                arguments = (address, bytes(self.buffer[8:8 + length]), bool(self.buffer[6]), self.buffer[1])
            try:
                frames.append(Frame(*arguments))
            except ValueError:
                del self.buffer[0]
                continue
            del self.buffer[:size]
        return frames


class SerialBus:
    """One serial connection bound to one CAN channel; receive filters the other.

    can_channel=None enables multiplexed SDK API use; frames then carry channels.

    SOULDE defaults to the fixed-eight layout emitted by the supplied SDK.
    Compact supports the manual's DLC=4 example. DLC=8 is identical in both.
    """
    def __init__(self, port='COM8', baudrate=115200, *, adapter_profile=None,
                 can_channel=1, packet_layout='fixed8', serial_port=None):
        require_profile(adapter_profile)
        require_layout(packet_layout)
        Frame(0, b'', False, 1 if can_channel is None else can_channel)
        if adapter_profile == 'at_shifted' and can_channel not in (None, 1):
            raise ValueError('AT profile has no CAN channel field')
        self.profile, self.can_channel, self.layout = adapter_profile, can_channel, packet_layout
        self.parser = PacketParser(adapter_profile, packet_layout=packet_layout)
        self.pending = []
        self._lock = threading.Lock()
        self._read_lock = threading.Lock()
        self.closed = False
        if serial_port is None:
            import serial
            options = {'exclusive': True} if os.name == 'posix' else {}
            serial_port = serial.Serial(port, baudrate, timeout=0.05, write_timeout=1, **options)
        self.serial = serial_port

    def send(self, message):
        channel = getattr(message, 'channel', 1) if self.can_channel is None else self.can_channel
        frame = Frame(message.arbitration_id, bytes(message.data), message.is_extended_id, channel)
        packet = encode_packet(frame, self.profile, packet_layout=self.layout)
        with self._lock:
            if self.closed:
                raise RuntimeError('Serial bus is closed')
            offset = 0
            while offset < len(packet):
                written = self.serial.write(packet[offset:])
                if type(written) is not int or not 0 < written <= len(packet) - offset:
                    raise OSError('Serial write made no progress')
                offset += written

    def recv(self, timeout=0.1):
        if not math.isfinite(timeout) or timeout < 0:
            raise ValueError('Receive timeout must be finite and nonnegative')
        deadline = time.monotonic() + timeout
        attempted = False
        with self._read_lock:
            while not self.closed:
                if self.pending:
                    return self.pending.pop(0)
                left = deadline - time.monotonic()
                available = min(getattr(self.serial, 'in_waiting', 0), 4096)
                if left <= 0 and (attempted or not available):
                    return None
                if hasattr(self.serial, 'timeout'):
                    self.serial.timeout = max(0, min(0.05, left))
                data = self.serial.read(max(1, available))
                attempted = True
                self.pending.extend(frame for frame in self.parser.feed(data)
                                    if self.can_channel is None or frame.channel == self.can_channel)
        return None

    def shutdown(self):
        with self._lock:
            self.closed = True
        with self._read_lock:
            if not getattr(self, '_serial_closed', False):
                self.serial.close()
                self._serial_closed = True
