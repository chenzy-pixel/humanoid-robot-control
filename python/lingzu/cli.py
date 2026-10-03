"""Explicit hardware entry point shared by experiment scripts."""
import argparse
import time
from .controller import LingzuMotorController
from .protocol import MODEL_LIMITS, bounded, control_frame


def run_can_demo(interface='soulde_usb2can', channel='/dev/USB2CAN0'):
    parser = argparse.ArgumentParser(description='Lingzu private CAN example')
    parser.add_argument('--interface', default=interface)
    parser.add_argument('--channel', default=channel)
    parser.add_argument('--can-channel', type=int, choices=[1, 2], default=1)
    parser.add_argument('--packet-layout', choices=['fixed8', 'compact'], default='fixed8')
    parser.add_argument('--model', choices=MODEL_LIMITS, required=True)
    parser.add_argument('--motor-id', type=int, default=1)
    parser.add_argument('--position', type=float, default=0)
    parser.add_argument('--min-position', type=float, required=True)
    parser.add_argument('--max-position', type=float, required=True)
    parser.add_argument('--kp', type=float, default=18)
    parser.add_argument('--kd', type=float, default=0.8)
    parser.add_argument('--duration', type=float, default=5)
    args = parser.parse_args()
    if args.min_position >= args.max_position or args.duration <= 0:
        parser.error('Use valid mechanical limits and positive duration')
    bounded(args.position, args.min_position, args.max_position)
    limits = MODEL_LIMITS[args.model]
    bounded(args.min_position, -limits.position, limits.position)
    bounded(args.max_position, -limits.position, limits.position)
    control_frame(args.motor_id, args.model, position=args.position, kp=args.kp, kd=args.kd)
    controller = LingzuMotorController(interface=args.interface, channel=args.channel,
                                      can_channel=args.can_channel, packet_layout=args.packet_layout)
    controller.add_motor(args.motor_id, args.model)
    try:
        deadline = time.monotonic() + args.duration
        last_feedback = time.monotonic()
        while time.monotonic() < deadline:
            controller.send_control_command(args.motor_id, 'motion', position=args.position, kp=args.kp, kd=args.kd)
            if controller.data_event.wait(0.02):
                state = controller.state_snapshot(args.motor_id)
                print(state)
                if state['fault']:
                    raise RuntimeError('Motor reported a fault')
                if state['valid']:
                    last_feedback = state['received_at']
                controller.data_event.clear()
            if time.monotonic() - last_feedback > 0.5:
                raise RuntimeError('Motor feedback timed out')
    except KeyboardInterrupt:
        pass
    finally:
        controller.close()
