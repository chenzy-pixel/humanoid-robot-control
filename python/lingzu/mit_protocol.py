"""RS02 MIT standard CAN frames from the bundled manual pages 31-34."""
import struct
from .protocol import Frame, bounded, float_to_uint, identifier


def mit_special(motor_id, final_byte, mode=255):
    identifier(motor_id, motor=True)
    if final_byte not in (0xFC, 0xFD, 0xFE, 0xFB) or not 0 <= mode <= 255:
        raise ValueError('Invalid MIT special command')
    return Frame(motor_id, bytes([255]) * 6 + bytes([mode, final_byte]), False)


def mit_control(motor_id, position=0, velocity=0, kp=0, kd=0, torque=0):
    identifier(motor_id, motor=True)
    p = float_to_uint(position, -12.57, 12.57, 16)
    v = float_to_uint(velocity, -44, 44, 12)
    k = float_to_uint(kp, 0, 500, 12)
    d = float_to_uint(kd, 0, 5, 12)
    t = float_to_uint(torque, -17, 17, 12)
    data = bytes([p >> 8, p & 255, v >> 4, ((v & 15) << 4) | (k >> 8), k & 255,
                  d >> 4, ((d & 15) << 4) | (t >> 8), t & 255])
    return Frame(motor_id, data, False)


def mit_velocity(motor_id, radians_per_second, current_limit):
    identifier(motor_id, motor=True)
    speed = bounded(radians_per_second, -44, 44)
    current = bounded(current_limit, 0, 23)
    return Frame((2 << 8) | motor_id, struct.pack('<ff', speed, current), False)
