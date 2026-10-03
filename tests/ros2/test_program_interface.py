"""Native ROS 2 service/action tests against the production node and a virtual PTY.

Run after sourcing the built Jazzy workspace. No physical ports are used.
"""
import math
import os
from pathlib import Path
import pty
import select
import struct
import subprocess
import tempfile
import threading
import time
import tty
import unittest

import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint
from usb2can_demo_lingzu.srv import JointCommand
from usb2can_demo_lingzu.msg import TrajectoryTask
import yaml


def crc8(data):
    value = 0xFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x8C if value & 1 else 0)
    return value


def encode(value, limit):
    return int((value + limit) * 65535 / (2 * limit))


class VirtualMotors:
    def __init__(self):
        self.master, self.slave = pty.openpty()
        tty.setraw(self.master)
        tty.setraw(self.slave)
        self.path = os.ttyname(self.slave)
        self.lock = threading.Lock()
        self.motors = {1: dict(position=0.1, velocity=0.0, state=0, frozen=False),
                       2: dict(position=-0.1, velocity=0.0, state=0, frozen=False)}
        self.stopped = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        buffer = bytearray()
        while not self.stopped.is_set():
            readable, _, _ = select.select([self.master], [], [], 0.005)
            if readable:
                buffer.extend(os.read(self.master, 4096))
            while len(buffer) >= 17:
                if buffer[0] != 0xA8 or crc8(buffer[:16]) != buffer[16]:
                    del buffer[0]
                    continue
                packet = bytes(buffer[:17]); del buffer[:17]
                can_id = struct.unpack('<I', packet[2:6])[0]
                kind, motor_id = can_id >> 24 & 31, can_id & 255
                with self.lock:
                    motor = self.motors[motor_id]
                    if kind == 3:
                        motor['state'] = 2
                    elif kind == 4:
                        motor['state'], motor['velocity'] = 0, 0.0
                    elif kind == 1 and not motor['frozen']:
                        position, velocity = struct.unpack('>HH', packet[8:12])
                        motor['position'] = position * 25.14 / 65535 - 12.57
                        motor['velocity'] = velocity * 66 / 65535 - 33
            with self.lock:
                for motor_id, motor in self.motors.items():
                    can_id = (2 << 24) | (motor['state'] << 22) | (motor_id << 8)
                    payload = struct.pack('>4H', encode(motor['position'], 12.57), encode(motor['velocity'], 33), 32767, 250)
                    packet = bytes([0xA9, 1]) + struct.pack('<I', can_id) + bytes([1, 8]) + payload
                    os.write(self.master, packet + bytes([crc8(packet)]))

    def freeze(self, motor_id):
        with self.lock:
            self.motors[motor_id]['frozen'] = True
            self.motors[motor_id]['velocity'] = 0.0

    def close(self):
        self.stopped.set(); self.thread.join(timeout=2)
        os.close(self.master); os.close(self.slave)


class NativeInterfaces(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='lingzu-native-')
        self.root = Path(self.directory.name)
        self.motors = VirtualMotors()
        self.namespace = '/lingzu_test_' + str(os.getpid()) + '_' + self._testMethodName
        self.node = rclpy.create_node('test_client', namespace=self.namespace)
        self.client = self.node.create_client(JointCommand, 'joint_command')
        self.action = ActionClient(self.node, FollowJointTrajectory, 'follow_joint_trajectory')
        self.tasks = []
        self.subscription = self.node.create_subscription(TrajectoryTask, 'trajectory_task', self.tasks.append, 10)
        self.pending_heartbeats = []
        self.ready = False
        self.timer = self.node.create_timer(0.05, self.heartbeat)
        self.log = (self.root/'controller.log').open('w')
        params = dict(hardware_confirmed=True, control_mode='trajectory', input_source='program',
                      device0=self.motors.path, device1='/nonexistent-unused-usb2can',
                      calibration_file=str(self.root/'calibration.txt'), log_file=str(self.root/'motor.csv'),
                      max_control_gap=0.2, input_timeout=0.4, feedback_timeout=0.3,
                      goal_settle_time=0.05, goal_timeout=0.4, tracking_error_timeout=0.1,
                      motor_names=['a', 'b'])
        for name, motor_id, direction, offset in [('a', 1, -1.0, 0.2), ('b', 2, 1.0, 0.0)]:
            fields = dict(device=0, channel=1, id=motor_id, model='RS00', min_position=-0.5,
                          max_position=0.5, neutral_position=0.0, direction=direction, zero_offset=offset,
                          kp=18.0, kd=0.8, max_velocity=0.5, max_acceleration=1.0, max_jerk=4.0,
                          follow_joystick=False)
            params.update({'motors.'+name+'.'+key: value for key, value in fields.items()})
        self.config = self.root/'params.yaml'
        self.config.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
        self.process = None
        self.addCleanup(self.cleanup)
        self.start_controller()

    def start_controller(self):
        self.ready = False
        self.heartbeat_acknowledged = False
        executable = Path(get_package_prefix('usb2can_demo_lingzu'))/'lib/usb2can_demo_lingzu/can_node'
        self.process = subprocess.Popen([str(executable), '--ros-args', '-r', '__ns:='+self.namespace,
                                         '--params-file', str(self.config)], stdout=self.log, stderr=subprocess.STDOUT)
        self.assertTrue(self.client.wait_for_service(timeout_sec=10), self.diagnostics())
        self.assertTrue(self.action.wait_for_server(timeout_sec=10), self.diagnostics())
        # Endpoint discovery may precede discovery of the client's reply reader.
        self.spin_for(0.5)
        self.ready = True
        deadline = time.monotonic() + 8
        while not self.heartbeat_acknowledged and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertTrue(self.heartbeat_acknowledged, self.diagnostics())

    def stop_controller(self):
        if self.process is not None:
            self.process.terminate()
            try:
                code = self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill(); self.process.wait(); raise
            self.assertEqual(code, 0, self.diagnostics())
            self.process = None

    def cleanup(self):
        try:
            self.timer.cancel()
            self.stop_controller()
        finally:
            self.action.destroy()
            self.node.destroy_node()
            self.motors.close()
            self.log.close()
            self.directory.cleanup()

    def diagnostics(self):
        self.log.flush()
        return (self.root/'controller.log').read_text()

    def heartbeat(self):
        if self.ready and self.client.service_is_ready():
            request = JointCommand.Request(); request.operation = 'heartbeat'
            for future in self.pending_heartbeats:
                if future.done() and future.exception() is None and future.result().success:
                    self.heartbeat_acknowledged = True
            self.pending_heartbeats = [f for f in self.pending_heartbeats if not f.done()]
            self.pending_heartbeats.append(self.client.call_async(request))

    def await_future(self, future, timeout=8):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        self.assertTrue(future.done(), self.diagnostics())
        return future.result()

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def command(self, operation, names=(), positions=()):
        request = JointCommand.Request(); request.operation = operation
        request.joint_names, request.positions = list(names), list(positions)
        return self.await_future(self.client.call_async(request))

    def send_goal(self, target=0.2, path_tolerance=None):
        goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = ['a']
        point = JointTrajectoryPoint(); point.positions = [target]; point.time_from_start.sec = 1
        goal.trajectory.points = [point]
        if path_tolerance is not None:
            tolerance = JointTolerance(); tolerance.name = 'a'; tolerance.position = path_tolerance
            goal.path_tolerance = [tolerance]
        self.feedback = []
        handle = self.await_future(self.action.send_goal_async(goal, feedback_callback=lambda f: self.feedback.append(f.feedback)))
        self.assertTrue(handle.accepted)
        return handle

    def test_success_and_joint_coordinate_feedback(self):
        self.assertFalse(self.command('enable', ['a', 'missing']).success)
        self.assertTrue(self.command('enable', ['a']).success)
        self.assertEqual(list(self.command('status').enabled), [True, False])
        result = self.await_future(self.send_goal().get_result_async())
        self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(result.result.error_code, 0)
        self.assertTrue(self.feedback)
        self.assertEqual(self.feedback[-1].joint_names, ['a'])
        self.assertAlmostEqual(self.feedback[-1].actual.positions[0], 0.2, delta=0.002)
        self.assertTrue(any(t.active and 0 < t.progress < 1 for t in self.tasks))
        self.spin_for(0.05)
        self.assertEqual(self.tasks[-1].state, 'completed')
        with self.motors.lock:
            self.assertAlmostEqual(self.motors.motors[1]['position'], 0.0, delta=0.002)
        self.assertTrue(self.command('enable', ['b']).success)
        self.assertTrue(self.command('disable', ['a']).success)
        self.assertEqual(list(self.command('status').enabled), [False, True])

    def test_stalled_path_aborts(self):
        self.assertTrue(self.command('enable', ['a']).success)
        self.motors.freeze(1)
        result = self.await_future(self.send_goal(path_tolerance=0.02).get_result_async())
        self.assertEqual(result.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(result.result.error_code, FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED)
        self.assertIn('a', result.result.error_string)
        self.assertFalse(self.command('status', ['a']).enabled[0])

    def test_goal_deadline_and_completed_requires_arrival(self):
        self.assertTrue(self.command('enable', ['a']).success)
        self.motors.freeze(1)
        future = self.send_goal().get_result_async()
        self.spin_for(1.15)
        self.assertFalse(future.done())
        self.assertTrue(any(t.state == 'settling' and t.active and t.progress == 1.0 for t in self.tasks))
        self.assertFalse(any(t.state == 'completed' for t in self.tasks))
        result = self.await_future(future)
        self.assertEqual(result.result.error_code, FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED)

    def test_cancel_and_preempt_report_terminal_results(self):
        self.assertTrue(self.command('enable', ['a']).success)
        handle = self.send_goal(); self.spin_for(0.2)
        cancellation = self.await_future(handle.cancel_goal_async())
        self.assertTrue(cancellation.goals_canceling)
        result = self.await_future(handle.get_result_async())
        self.assertEqual(result.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(result.result.error_code, 0)
        first = self.send_goal(); self.spin_for(0.15)
        replacement = self.send_goal(0.15)
        self.assertEqual(self.await_future(first.get_result_async()).status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(self.await_future(replacement.get_result_async()).status, GoalStatus.STATUS_SUCCEEDED)

    def test_calibration_persistence_and_zero(self):
        calibration = self.command('calibrate', ['a'], [0.25])
        self.assertTrue(calibration.success, calibration.message)
        self.assertAlmostEqual(calibration.positions[0], 0.25, delta=0.002)
        self.assertAlmostEqual(calibration.zero_offsets[0], 0.35, delta=0.002)
        self.assertFalse(self.command('calibrate', ['a'], [10.0]).success)
        self.stop_controller(); self.spin_for(0.2); self.start_controller()
        loaded = self.command('status', ['a'])
        self.assertAlmostEqual(loaded.zero_offsets[0], calibration.zero_offsets[0])
        self.assertTrue(self.command('enable', ['a']).success)
        self.assertFalse(self.command('zero', ['a']).success)
        self.assertTrue(self.command('disable', ['a']).success)
        self.assertTrue(self.command('zero', ['a']).success)
        self.assertAlmostEqual(self.command('status', ['a']).positions[0], 0.0, delta=0.002)

    def test_program_heartbeat_timeout_aborts_task(self):
        self.assertTrue(self.command('enable', ['a']).success)
        handle = self.send_goal(); self.timer.cancel()
        result = self.await_future(handle.get_result_async())
        self.assertEqual(result.status, GoalStatus.STATUS_ABORTED)
        self.assertIn('heartbeat', result.result.error_string)


if __name__ == '__main__':
    unittest.main(verbosity=2)
