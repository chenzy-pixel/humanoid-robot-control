"""Lingzu private CAN protocol, matching the bundled RS00/02/03/04 manuals."""
from dataclasses import dataclass
import math
import struct


@dataclass(frozen=True)
class Limits:
    position: float
    velocity: float
    torque: float
    kp: float
    kd: float
    current: float
    speed_mode: float


MODEL_LIMITS = {
    'RS00': Limits(12.57, 33, 14, 500, 5, 16, 33),
    'RS02': Limits(4 * math.pi, 44, 17, 500, 5, 23, 44),
    'RS03': Limits(12.57, 20, 60, 5000, 100, 43, 20),
    'RS04': Limits(12.57, 15, 120, 5000, 100, 90, 20),
}


@dataclass(frozen=True)
class Frame:
    arbitration_id: int
    data: bytes
    is_extended_id: bool = True
    channel: int = 1

    def __post_init__(self):
        maximum = 0x1FFFFFFF if self.is_extended_id else 0x7FF
        if not isinstance(self.arbitration_id, int) or not 0 <= self.arbitration_id <= maximum:
            raise ValueError('CAN ID outside frame range')
        if len(self.data) > 8:
            raise ValueError('Classic CAN supports at most 8 data bytes')
        if type(self.channel) is not int or self.channel not in (1, 2):
            raise ValueError('USB2CAN channel must be 1 or 2')

    @property
    def dlc(self):
        return len(self.data)


def model_limits(model):
    try:
        return MODEL_LIMITS[model]
    except KeyError:
        raise ValueError('Select motor model RS00/RS02/RS03/RS04') from None


def identifier(value, *, motor=False):
    if type(value) is not int or not (1 <= value <= 254 if motor else 0 <= value <= 255):
        raise ValueError('Motor ID must be 1..254; host ID must be 0..255')
    return value


def bounded(value, low, high):
    value = float(value)
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'Expected a finite value in [{low}, {high}]')
    return value


def float_to_uint(value, low, high, bits=16):
    if not math.isfinite(low) or not math.isfinite(high) or high <= low or bits not in (12, 16):
        raise ValueError('Invalid quantization range')
    return int((bounded(value, low, high) - low) * ((1 << bits) - 1) / (high - low))


def uint_to_float(value, low, high, bits=16):
    return value * (high - low) / ((1 << bits) - 1) + low


def control_frame(motor_id, model, position=0, velocity=0, kp=0, kd=0, torque=0):
    identifier(motor_id, motor=True)
    p = model_limits(model)
    values = [float_to_uint(position, -p.position, p.position),
              float_to_uint(velocity, -p.velocity, p.velocity),
              float_to_uint(kp, 0, p.kp), float_to_uint(kd, 0, p.kd)]
    exdata = float_to_uint(torque, -p.torque, p.torque)
    return Frame((1 << 24) | (exdata << 8) | motor_id, struct.pack('>4H', *values))


def special_frame(comm_type, motor_id, host_id=0, flag=False):
    if comm_type not in (0, 3, 4, 6):
        raise ValueError('Unsupported special command')
    identifier(motor_id, motor=True)
    identifier(host_id)
    return Frame((comm_type << 24) | (host_id << 8) | motor_id, bytes([int(flag)]) + bytes(7))


PARAMETER_TYPES = {0x7005: 'B', 0x7024: 'H'}
PARAMETER_TYPES.update({index: 'f' for index in (
    0x7006, 0x700A, 0x700B, 0x7010, 0x7011, 0x7014,
    0x7016, 0x7017, 0x7018, 0x7019, 0x701A, 0x701B,
    0x701C, 0x701E, 0x701F, 0x7020)})


def parameter_frame(motor_id, host_id, index, value=None):
    identifier(motor_id, motor=True)
    identifier(host_id)
    if type(index) is not int or not 0 <= index <= 65535:
        raise ValueError('Parameter index must be uint16')
    payload = struct.pack('<H', index) + bytes(2)
    if value is None:
        return Frame((0x11 << 24) | (host_id << 8) | motor_id, payload + bytes(4))
    fmt = PARAMETER_TYPES.get(index)
    if fmt is None:
        raise ValueError('Parameter write requires a known data type')
    if fmt == 'f' and not math.isfinite(float(value)):
        raise ValueError('Non-finite parameter')
    try:
        raw = struct.pack('<' + fmt, value)
    except (OverflowError, struct.error) as error:
        raise ValueError('Parameter value is not representable by its wire type') from error
    return Frame((0x12 << 24) | (host_id << 8) | motor_id, payload + raw.ljust(4, b'\0'))


def valid_message(message, comm_type, host_id=None):
    return (message is not None and getattr(message, 'is_extended_id', False)
            and not getattr(message, 'is_error_frame', False)
            and not getattr(message, 'is_remote_frame', False)
            and 0 <= message.arbitration_id <= 0x1FFFFFFF
            and ((message.arbitration_id >> 24) & 31) == comm_type
            and len(message.data) == 8 and getattr(message, 'dlc', 8) == 8
            and (host_id is None or message.arbitration_id & 255 == host_id))


def decode_feedback(message, model, host_id=None):
    if not valid_message(message, 2, host_id):
        return None
    p = model_limits(model)
    motor_id = (message.arbitration_id >> 8) & 255
    if not 1 <= motor_id <= 254:
        return None
    position, velocity, torque, temperature = struct.unpack('>4H', message.data)
    fault = (message.arbitration_id >> 16) & 63
    return {'motor_id': motor_id, 'host_id': message.arbitration_id & 255,
            'position': uint_to_float(position, -p.position, p.position),
            'velocity': uint_to_float(velocity, -p.velocity, p.velocity),
            'torque': uint_to_float(torque, -p.torque, p.torque),
            'temperature': temperature / 10, 'fault': fault, 'fault_info': bin(fault),
            'motor_state': (message.arbitration_id >> 22) & 3, 'valid': True}
