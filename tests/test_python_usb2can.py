"""Protocol vectors and SDK facade regressions; no real ports or native DLLs."""
import ctypes
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))
from lingzu.protocol import Frame
from lingzu.serial_transport import PacketParser, SerialBus, crc8, encode_packet
from lingzu.controller import LingzuMotorController
from pyusb2can import USB2CAN, FrameInfo, STANDARD, EXTENDED
from usb2can_fakes import Serial, feedback

PROFILE = 'soulde_usb2can_v25'


def received(frame, layout='fixed8'):
    packet = bytearray(encode_packet(frame, PROFILE, packet_layout=layout))
    packet[0] = 0xA9
    packet[-1] = crc8(packet[:-1])
    return bytes(packet)


class USB2CANTests(unittest.TestCase):
    def test_manual_compact_vector(self):
        frame = Frame(0x7FF, bytes.fromhex('FF FF 00 82'), False)
        self.assertEqual(encode_packet(frame, PROFILE, packet_layout='compact'),
                         bytes.fromhex('A8 01 FF 07 00 00 00 04 FF FF 00 82 E1'))

    def test_fixed8_zero_padding_crc(self):
        packet = encode_packet(Frame(0x7FF, bytes.fromhex('FF FF 00 82'), False), PROFILE)
        self.assertEqual(packet, bytes.fromhex('A8 01 FF 07 00 00 00 04 FF FF 00 82 00 00 00 00 C6'))

    def test_all_dlcs_and_channels(self):
        for layout in ['compact', 'fixed8']:
            for length in range(9):
                for channel in (1, 2):
                    frame = Frame(0x1ABCDEF, bytes(range(length)), True, channel)
                    self.assertEqual(PacketParser(PROFILE, packet_layout=layout).feed(received(frame, layout)), [frame])

    def test_every_split_and_coalesced_frame(self):
        a, b = Frame(0x500, b'\xA9' * 8, False, 2), Frame(0x1200FD01, bytes(8))
        stream = received(a) + received(b)
        for split in range(len(stream) + 1):
            parser = PacketParser(PROFILE)
            self.assertEqual(parser.feed(stream[:split]) + parser.feed(stream[split:]), [a, b])

    def test_bad_crc_resynchronizes(self):
        frame = Frame(0x501, bytes(8), False)
        broken = bytearray(received(frame)); broken[-1] ^= 1
        self.assertEqual(PacketParser(PROFILE).feed(b'noise' + broken + received(frame)), [frame])

    def test_invalid_headers_and_id_filtered(self):
        good = received(Frame(0x501, bytes(8), False))
        for offset, value in [(0, 0xA8), (1, 3), (6, 2), (7, 9), (4, 0x10)]:
            broken = bytearray(good); broken[offset] = value; broken[-1] = crc8(broken[:-1])
            self.assertEqual(PacketParser(PROFILE).feed(broken + good), [Frame(0x501, bytes(8), False)])

    def test_noise_does_not_grow_parser_buffer(self):
        parser = PacketParser(PROFILE)
        parser.feed(bytes(100000))
        self.assertEqual(parser.buffer, b'')

    def test_bound_channel_filters_same_motor_id(self):
        one = feedback(); two = Frame(one.arbitration_id, one.data, True, 2)
        serial = Serial(received(two) + received(one), partial_write=3)
        bus = SerialBus(serial_port=serial, adapter_profile=PROFILE, can_channel=1)
        self.assertEqual(bus.recv(), one)
        self.assertIsNone(bus.recv(.001))
        bus.send(one)
        self.assertEqual(serial.output, encode_packet(one, PROFILE))
        bus.shutdown(); bus.shutdown()
        self.assertTrue(serial.closed)

    def test_channel_two_transmit(self):
        serial = Serial()
        bus = SerialBus(serial_port=serial, adapter_profile=PROFILE, can_channel=2)
        bus.send(feedback())
        self.assertEqual(serial.output[1], 2)
        bus.shutdown()

    def test_nonblocking_receive_reads_available_packet(self):
        frame = feedback()
        bus = SerialBus(serial_port=Serial(received(frame)), adapter_profile=PROFILE)
        self.assertEqual(bus.recv(0), frame)
        self.assertIsNone(bus.recv(0))
        bus.shutdown()

    def test_profile_channel_layout_validate_before_open(self):
        for values in [dict(can_channel=0), dict(can_channel=True), dict(packet_layout='guess')]:
            with self.assertRaises(ValueError):
                SerialBus(adapter_profile=PROFILE, **values)
        with self.assertRaises(ValueError):
            SerialBus(adapter_profile='at_shifted', can_channel=2)

    def test_timeout_and_write_failures(self):
        serial = Serial()
        bus = SerialBus(serial_port=serial, adapter_profile=PROFILE)
        for timeout in (-1, float('inf'), float('nan')):
            with self.assertRaises(ValueError): bus.recv(timeout)
        serial.write = lambda data: 0
        with self.assertRaises(OSError): bus.send(feedback())
        bus.shutdown()
        with self.assertRaises(RuntimeError): bus.send(feedback())

    def test_sdk_structure_packing_and_constants(self):
        self.assertEqual(ctypes.sizeof(FrameInfo), 6)
        self.assertEqual(FrameInfo.frameType.offset, 4)
        self.assertEqual(FrameInfo.dataLength.offset, 5)
        self.assertEqual((STANDARD, EXTENDED), (0, 1))

    def test_sdk_api_receives_both_channels_and_actual_dlc(self):
        a, b = Frame(0x123, b'1234', False, 2), Frame(0x456, b'', False, 1)
        serial = Serial(received(a) + received(b), partial_write=2)
        with USB2CAN('unused', serial_port=serial) as can:
            channel, info, data = can.read_usbcan()
            self.assertEqual((channel, info.canID, info.frameType, info.dataLength, data), (2, 0x123, 0, 4, b'1234'))
            self.assertEqual(can.read_usbcan()[2], b'')
            self.assertIsNone(can.read_usbcan(1000))
            self.assertEqual(can.send_usbcan(2, FrameInfo(0x123, 0, 4), b'1234'), 17)
            self.assertEqual(serial.output, encode_packet(a, PROFILE))
        can.close()
        self.assertTrue(serial.closed)
        with self.assertRaises(RuntimeError): can.read_usbcan()

    def test_sdk_api_rejects_invalid_arguments(self):
        with USB2CAN('unused', serial_port=Serial()) as can:
            for channel, info, data in [(3, FrameInfo(1, 0, 1), b'1'),
                                        (1, FrameInfo(1, 2, 1), b'1'),
                                        (1, FrameInfo(1, 0, 2), b'1')]:
                with self.assertRaises(ValueError): can.send_usbcan(channel, info, data)
            for timeout in [-1, 1.0, 0x80000000]:
                with self.assertRaises(ValueError): can.read_usbcan(timeout)

    def test_controller_selects_soulde_without_python_can(self):
        fake = SerialBus(serial_port=Serial(), adapter_profile=PROFILE, can_channel=2)
        with patch('lingzu.serial_transport.SerialBus', return_value=fake) as create:
            controller = LingzuMotorController(interface='soulde_usb2can', channel='/dev/USB2CAN0',
                                                can_channel=2, start_listener=False)
            create.assert_called_once_with('/dev/USB2CAN0', adapter_profile=PROFILE, can_channel=2, packet_layout='fixed8')
            controller.add_motor(1, 'RS04')
            controller.close()
        with self.assertRaises(ValueError):
            LingzuMotorController(interface='soulde_usb2can', bitrate=500000, start_listener=False)

    def test_default_controller_matches_confirmed_board(self):
        fake = SerialBus(serial_port=Serial(), adapter_profile=PROFILE)
        with patch('lingzu.serial_transport.SerialBus', return_value=fake) as create:
            controller = LingzuMotorController(start_listener=False)
            create.assert_called_once_with('/dev/USB2CAN0', adapter_profile=PROFILE,
                                          can_channel=1, packet_layout='fixed8')
            controller.close()


if __name__ == '__main__':
    unittest.main()
