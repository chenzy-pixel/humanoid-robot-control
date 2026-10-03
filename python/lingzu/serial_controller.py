"""Private Lingzu commands over a deliberately selected serial adapter profile."""
import time
from .compat import MotorController as CANMotorController
from .protocol import special_frame, valid_message, model_limits, identifier
from .serial_transport import SerialBus


class MotorController(CANMotorController):
    def __init__(self, port='COM8', motor_id=1, *, model='RS02', host_id=0xFD,
                 adapter_profile=None, can_channel=1, packet_layout='fixed8', bus=None):
        model_limits(model)
        identifier(motor_id, motor=True)
        identifier(host_id)
        bus = bus if bus is not None else SerialBus(port, adapter_profile=adapter_profile,
                                                   can_channel=can_channel, packet_layout=packet_layout)
        try:
            super().__init__(model=model, host_id=host_id, bus=bus)
            self.motor_id = motor_id
            self.add_motor(motor_id, model)
        except BaseException:
            bus.shutdown()
            raise

    def send_enable(self, motor_id=None, master_id=None):
        return super().send_enable(self.motor_id if motor_id is None else motor_id, master_id)

    enable = send_enable

    def enable_damped(self):
        # Hybrid mode with kp=0 gives velocity damping without position stiffness.
        self.send_control_command(self.motor_id, 0, 0, 0, 2, 0)

    def set_velocity(self, rpm, current_limit=None):
        kwargs = {'speed': self.rpm_to_rads(rpm)}
        if current_limit is not None:
            kwargs['current_limit'] = current_limit
        # Call the multi-mode base implementation rather than the example hybrid API.
        from .controller import LingzuMotorController
        LingzuMotorController.send_control_command(self, self.motor_id, 'speed', **kwargs)

    send_velocity = set_velocity


class MotorScanner:
    def __init__(self, port='COM8', *, adapter_profile=None, bus=None, host_id=0xFD,
                 can_channel=1, packet_layout='fixed8'):
        self.port, self.profile, self.bus, self.host_id = port, adapter_profile, bus, host_id
        self.can_channel, self.packet_layout = can_channel, packet_layout
        self.motor_ids = []

    def scan(self, timeout=3.0, ids=range(1, 255)):
        if timeout <= 0:
            raise ValueError('Positive scan timeout required')
        bus = self.bus if self.bus is not None else SerialBus(self.port, adapter_profile=self.profile,
                                                             can_channel=self.can_channel, packet_layout=self.packet_layout)
        found = set()
        try:
            for motor_id in ids:
                bus.send(special_frame(0, motor_id, self.host_id))
                time.sleep(0.0003)
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                frame = bus.recv(timeout=max(0, deadline - time.monotonic()))
                if frame is None:
                    break
                if valid_message(frame, 0) and frame.arbitration_id & 255 == 0xFE:
                    motor_id = (frame.arbitration_id >> 8) & 255
                    if 1 <= motor_id <= 254:
                        found.add(motor_id)
            self.motor_ids = sorted(found)
            return list(self.motor_ids)
        finally:
            if self.bus is None:
                bus.shutdown()


def run_serial_demo():
    import argparse
    from .protocol import MODEL_LIMITS
    parser = argparse.ArgumentParser(description='Private Lingzu CAN via an explicit USB2CAN profile')
    parser.add_argument('--port', required=True)
    parser.add_argument('--adapter-profile', choices=['soulde_usb2can_v25', 'at_shifted'], required=True)
    parser.add_argument('--can-channel', type=int, choices=[1, 2], default=1)
    parser.add_argument('--packet-layout', choices=['fixed8', 'compact'], default='fixed8')
    parser.add_argument('--model', choices=MODEL_LIMITS, required=True)
    parser.add_argument('--motor-id', type=int, default=1)
    parser.add_argument('--rpm', type=float, default=0)
    parser.add_argument('--current-limit', type=float, required=True)
    parser.add_argument('--duration', type=float, default=5)
    parser.add_argument('--scan', action='store_true')
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error('Positive duration required')
    if args.scan:
        print(MotorScanner(args.port, adapter_profile=args.adapter_profile,
                           can_channel=args.can_channel, packet_layout=args.packet_layout).scan())
        return
    controller = MotorController(args.port, args.motor_id, model=args.model, adapter_profile=args.adapter_profile,
                                 can_channel=args.can_channel, packet_layout=args.packet_layout)
    try:
        controller.set_velocity(args.rpm, args.current_limit)
        deadline = time.monotonic() + args.duration
        last_feedback = time.monotonic()
        while time.monotonic() < deadline:
            feedback = controller.receive_feedback()
            if feedback is not None:
                print(feedback)
                last_feedback = time.monotonic()
                if feedback['fault']:
                    raise RuntimeError('Motor reported a fault')
            if time.monotonic() - last_feedback > 0.5:
                raise RuntimeError('Motor feedback timed out')
    except KeyboardInterrupt:
        pass
    finally:
        controller.close()
