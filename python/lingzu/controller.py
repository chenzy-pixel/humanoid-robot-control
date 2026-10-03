"""Synchronous CAN reception with optional worker thread and injectable bus."""
import logging
import math
import struct
import threading
import time
from .protocol import (Frame, bounded, control_frame, decode_feedback, identifier,
                       model_limits, parameter_frame, special_frame, valid_message, PARAMETER_TYPES)


class MotorNode:
    def __init__(self, motor_id, motor_type, host_id):
        self.motor_id = identifier(motor_id, motor=True)
        self.host_id = identifier(host_id)
        self.type = motor_type
        self.limits = p = model_limits(motor_type)
        self.max_iq, self.max_speed, self.max_position = p.current, p.velocity, p.position
        self.max_kp, self.max_kd = p.kp, p.kd
        self.pos_scale, self.pos_offset = 2 * p.position, p.position
        self.vel_scale, self.vel_offset = 2 * p.velocity, p.velocity
        self.torque_scale, self.torque_offset = 2 * p.torque, p.torque
        self.run_mode = None
        self.enabled = False
        self.parameters = {}
        self.state = dict(position=None, velocity=None, torque=None, temperature=None,
                          fault=0, warning=0, motor_state=None, valid=False, received_at=None)


class LingzuMotorController:
    def __init__(self, host_id=0xFD, channel='/dev/USB2CAN0', interface='soulde_usb2can', bitrate=1000000,
                 *, bus=None, start_listener=True, message_factory=None, bustype=None,
                 can_channel=1, packet_layout='fixed8'):
        self.host_id = identifier(host_id)
        self.motors = {}
        self._lock = threading.RLock()
        self._send_lock = threading.Lock()
        self._stop = threading.Event()
        self.data_event = threading.Event()
        self.logger = logging.getLogger('LingzuController')
        self._next_send = 0.0
        self._message_factory = message_factory or Frame
        self.running = True
        self.recv_thread = None
        if bus is None:
            selected = bustype or interface
            if selected == 'soulde_usb2can':
                if bitrate != 1000000:
                    raise ValueError('Classic USB2CAN CAN speed is fixed at 1 Mbit/s')
                from .serial_transport import SerialBus
                bus = SerialBus(channel, adapter_profile='soulde_usb2can_v25',
                                can_channel=can_channel, packet_layout=packet_layout)
            else:
                import can
                bus = can.Bus(interface=selected, channel=channel, bitrate=bitrate)
                self._message_factory = can.Message
        self.bus = bus
        if start_listener:
            try:
                self.recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
                self.recv_thread.start()
            except BaseException:
                self.bus.shutdown()
                raise

    def add_motor(self, motor_id, motor_type='RS02'):
        with self._lock:
            if motor_id in self.motors:
                raise ValueError('Duplicate motor ID')
            motor = MotorNode(motor_id, motor_type, self.host_id)
            self.motors[motor_id] = motor
            return motor

    def _motor(self, motor_id):
        with self._lock:
            if motor_id not in self.motors:
                raise ValueError(f'Motor {motor_id} has not been registered')
            return self.motors[motor_id]

    def _send(self, frame, attempts=1):
        if not self.running:
            raise RuntimeError('Controller is closed')
        with self._send_lock:
            for attempt in range(attempts):
                time.sleep(max(0, self._next_send - time.monotonic()))
                try:
                    message = self._message_factory(arbitration_id=frame.arbitration_id,
                                                    data=frame.data, is_extended_id=frame.is_extended_id)
                    self.bus.send(message)
                    return
                except Exception:
                    if attempt + 1 == attempts:
                        raise
                finally:
                    self._next_send = time.monotonic() + 0.0003

    def enable_motor(self, motor_id):
        motor = self._motor(motor_id)
        self._send(special_frame(3, motor_id, self.host_id))
        motor.enabled = True

    def disable_motor(self, motor_id):
        motor = self._motor(motor_id)
        try:
            self._send(special_frame(4, motor_id, self.host_id), attempts=3)
        finally:
            motor.enabled = False

    send_emergency_stop = disable_motor

    def set_mechanical_zero(self, motor_id):
        self.disable_motor(motor_id)
        self._send(special_frame(6, motor_id, self.host_id, flag=True))

    def set_motor_mode(self, motor_id, mode):
        if type(mode) is not int or mode not in (0, 1, 2, 3, 5):
            raise ValueError('Invalid run mode')
        motor = self._motor(motor_id)
        self.disable_motor(motor_id)
        self._send(parameter_frame(motor_id, self.host_id, 0x7005, mode))
        motor.run_mode = mode

    def _ensure_mode(self, motor, mode):
        if motor.run_mode != mode:
            self.set_motor_mode(motor.motor_id, mode)
        if not motor.enabled:
            if mode in (2, 3):
                index = 0x700A if mode == 2 else 0x7006
                self._send(parameter_frame(motor.motor_id, self.host_id, index, 0.0))
            self.enable_motor(motor.motor_id)

    def send_control_command(self, motor_id, control_mode, **kwargs):
        motor = self._motor(motor_id)
        # Validate the complete command before any mode change or enable write.
        if control_mode in ('position', 'motion'):
            frame = control_frame(motor_id, motor.type, kwargs.get('position', 0),
                                  kwargs.get('speed', kwargs.get('velocity', 0)),
                                  kwargs.get('kp', 0), kwargs.get('kd', 0), kwargs.get('torque', 0))
            self._ensure_mode(motor, 0)
            self._send(frame)
        elif control_mode == 'current':
            value = bounded(kwargs.get('iq', 0), -motor.max_iq, motor.max_iq)
            self._ensure_mode(motor, 3)
            self._send(parameter_frame(motor_id, self.host_id, 0x7006, value))
        elif control_mode == 'speed':
            value = bounded(kwargs.get('speed', 0), -motor.limits.speed_mode, motor.limits.speed_mode)
            current = bounded(kwargs.get('current_limit', motor.max_iq), 0, motor.max_iq)
            gain = None
            if 'kp' in kwargs:
                gain = float(kwargs['kp'])
                if not math.isfinite(gain) or gain < 0:
                    raise ValueError('Speed kp must be finite and nonnegative')
            gain_frame = parameter_frame(motor_id, self.host_id, 0x701F, gain) if gain is not None else None
            self._ensure_mode(motor, 2)
            self._send(parameter_frame(motor_id, self.host_id, 0x7018, current))
            if gain is not None:
                self._send(gain_frame)
            self._send(parameter_frame(motor_id, self.host_id, 0x700A, value))
        else:
            raise ValueError('Use motion/position (hybrid), current or speed')

    def read_parameter(self, motor_id, index):
        self._motor(motor_id)
        self._send(parameter_frame(motor_id, self.host_id, index))

    def _parse_parameter_response(self, motor, data, result=0):
        index = struct.unpack('<H', data[:2])[0]
        fmt = PARAMETER_TYPES.get(index)
        raw = bytes(data[4:8])
        value = struct.unpack('<' + fmt, raw[:struct.calcsize(fmt)])[0] if fmt else raw
        motor.parameters[index] = {'result': result, 'raw': bytes(data[4:8]), 'value': value if result == 0 else None}

    def _parse_fault_frame(self, motor, data):
        motor.state['fault'], motor.state['warning'] = struct.unpack('<II', data)

    def _parse_can_frame(self, message):
        if not getattr(message, 'is_extended_id', False):
            return
        kind = (message.arbitration_id >> 24) & 31
        if kind not in (2, 0x11, 0x15) or not valid_message(message, kind, self.host_id):
            return
        motor_id = (message.arbitration_id >> 8) & 255
        with self._lock:
            motor = self.motors.get(motor_id)
            if motor is None:
                return
            if kind == 2:
                state = decode_feedback(message, motor.type, self.host_id)
                if state is None:
                    return
                state['received_at'] = time.monotonic()
                motor.state.update(state)
            elif kind == 0x15:
                self._parse_fault_frame(motor, message.data)
            else:
                self._parse_parameter_response(motor, message.data, (message.arbitration_id >> 16) & 255)
            self.data_event.set()

    def state_snapshot(self, motor_id):
        with self._lock:
            return dict(self._motor(motor_id).state)

    def _recv_loop(self):
        while not self._stop.is_set():
            try:
                message = self.bus.recv(timeout=0.1)
                if message is not None:
                    self._parse_can_frame(message)
            except Exception as error:
                self.logger.error('CAN receive failed: %s', error)
                self._stop.wait(0.05)

    def close(self):
        if not self.running:
            return
        errors = []
        try:
            with self._lock:
                motors = list(self.motors.values())
            for motor in motors:
                try:
                    self.disable_motor(motor.motor_id)
                except Exception as error:
                    errors.append(error)
        finally:
            self.running = False
            self._stop.set()
            if self.recv_thread is not None:
                self.recv_thread.join(timeout=1.0)
            self.bus.shutdown()
        if errors:
            raise RuntimeError('Motor stop transmission failed during shutdown') from errors[0]

    shutdown = close

    @staticmethod
    def rpm_to_rads(rpm):
        return rpm * 2 * math.pi / 60
