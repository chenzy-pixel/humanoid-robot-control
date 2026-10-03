"""Single-motor example API backed by the shared codec; more models may be registered."""
import time
from .controller import LingzuMotorController
from .protocol import decode_feedback, float_to_uint, identifier, model_limits


class MotorController(LingzuMotorController):
    def __init__(self, channel='PCAN_USBBUS1', interface='pcan', bitrate=1000000,
                 *, model='RS02', host_id=0, bus=None, bustype=None):
        p = model_limits(model)
        self.model = model
        self.P_MIN, self.P_MAX = -p.position, p.position
        self.V_MIN, self.V_MAX = -p.velocity, p.velocity
        self.T_MIN, self.T_MAX = -p.torque, p.torque
        self.KP_MIN, self.KP_MAX = 0, p.kp
        self.KD_MIN, self.KD_MAX = 0, p.kd
        super().__init__(host_id, channel, bustype or interface, bitrate, bus=bus, start_listener=False)

    float_to_uint = staticmethod(float_to_uint)

    def _register(self, motor_id, master_id=None):
        identifier(motor_id, motor=True)
        if master_id is not None and master_id != self.host_id:
            raise ValueError('master_id must match the controller host_id')
        if motor_id not in self.motors:
            self.add_motor(motor_id, self.model)

    def send_enable(self, motor_id, master_id=None):
        self._register(motor_id, master_id)
        self.enable_motor(motor_id)

    def send_stop(self, motor_id, master_id=None):
        self._register(motor_id, master_id)
        self.disable_motor(motor_id)

    def send_control_command(self, motor_id, position, velocity, kp, kd, torque, master_id=None):
        self._register(motor_id, master_id)
        super().send_control_command(motor_id, 'motion', position=position, velocity=velocity,
                                     kp=kp, kd=kd, torque=torque)

    def receive_feedback(self, timeout=0.1):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            message = self.bus.recv(timeout=max(0, deadline - time.monotonic()))
            if message is None:
                return None
            motor = self.motors.get((message.arbitration_id >> 8) & 255)
            result = decode_feedback(message, motor.type if motor else self.model, self.host_id)
            if result is not None:
                self._parse_can_frame(message)
                return result
        return None
