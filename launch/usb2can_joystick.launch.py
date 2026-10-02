"""Start the ROS 2 SDL joystick driver and USB2CAN controller."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('usb2can_demo_lingzu')
    motor_config = LaunchConfiguration('motor_config')
    hardware_confirmed = LaunchConfiguration('hardware_confirmed')
    device_id = LaunchConfiguration('joystick_device_id')
    device_name = LaunchConfiguration('joystick_device_name')
    log_file = LaunchConfiguration('log_file')
    control_mode = LaunchConfiguration('control_mode')
    retime_trajectory = LaunchConfiguration('retime_trajectory')
    joystick = Node(
        package='joy', executable='joy_node', name='joy_node', output='screen',
        parameters=[{
            'device_id': ParameterValue(device_id, value_type=int),
            'device_name': ParameterValue(device_name, value_type=str),
            'autorepeat_rate': 20.0, 'deadzone': 0.0, 'sticky_buttons': False,
        }],
    )
    controller = Node(
        package='usb2can_demo_lingzu', executable='can_node',
        name='can_motor_controller_node', output='screen',
        parameters=[motor_config, {
            'hardware_confirmed': ParameterValue(hardware_confirmed, value_type=bool),
            'log_file': ParameterValue(log_file, value_type=str),
            'control_mode': ParameterValue(control_mode, value_type=str),
            'retime_trajectory': ParameterValue(retime_trajectory, value_type=bool),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument('motor_config', default_value=os.path.join(share, 'config', 'motors.yaml')),
        DeclareLaunchArgument('hardware_confirmed', default_value='false'),
        DeclareLaunchArgument('control_mode', default_value='joystick'),
        DeclareLaunchArgument('retime_trajectory', default_value='false'),
        DeclareLaunchArgument('joystick_device_id', default_value='0'),
        DeclareLaunchArgument('joystick_device_name', default_value=''),
        DeclareLaunchArgument('log_file', default_value='motor_angle_log.csv'),
        RegisterEventHandler(OnProcessExit(
            target_action=controller,
            on_exit=[EmitEvent(event=Shutdown(reason='Motor controller exited'))],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=joystick,
            on_exit=[EmitEvent(event=Shutdown(reason='Joystick driver exited'))],
        )),
        joystick, controller,
    ])
