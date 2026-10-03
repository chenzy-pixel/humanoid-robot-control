"""SDK-compatible Python API using the checked serial codec.

The supplied native libraries differ in send return values and skip RX checks.
This implementation shares the project's CRC and stream handling instead.
"""
from ctypes import Structure, c_uint32, c_uint8
from lingzu.protocol import Frame
from lingzu.serial_transport import SerialBus, encode_packet

STANDARD = 0
EXTENDED = 1


class FrameInfo(Structure):
    _pack_ = 1
    _fields_ = [('canID', c_uint32), ('frameType', c_uint8), ('dataLength', c_uint8)]


class USB2CAN:
    def __init__(self, device_name, *, packet_layout='fixed8', serial_port=None):
        self._bus = SerialBus(device_name, adapter_profile='soulde_usb2can_v25',
                              can_channel=None, packet_layout=packet_layout, serial_port=serial_port)
        self.closed = False

    def send_usbcan(self, channel, info, data):
        if self.closed:
            raise RuntimeError('USB2CAN device is closed')
        if info.frameType not in (STANDARD, EXTENDED) or len(data) != info.dataLength:
            raise ValueError('Invalid frame type or dataLength does not match data')
        frame = Frame(info.canID, bytes(data), info.frameType == EXTENDED, channel)
        self._bus.send(frame)
        return len(encode_packet(frame, self._bus.profile, packet_layout=self._bus.layout))

    def read_usbcan(self, timeout=100000):
        if self.closed:
            raise RuntimeError('USB2CAN device is closed')
        if type(timeout) is not int or not 0 <= timeout <= 0x7FFFFFFF:
            raise ValueError('SDK timeout must be a nonnegative int32 in microseconds')
        frame = self._bus.recv(timeout / 1000000)
        if frame is None:
            return None
        return frame.channel, FrameInfo(frame.arbitration_id, int(frame.is_extended_id), frame.dlc), bytearray(frame.data)

    def close(self):
        if not self.closed:
            self.closed = True
            self._bus.shutdown()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
