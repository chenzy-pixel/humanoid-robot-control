"""Evaluate launch construction using inert launch API substitutes."""
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]


class Action:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs


class LaunchDescription:
    def __init__(self, entities):
        self.entities = entities


def module(name, **values):
    result = types.ModuleType(name)
    result.__dict__.update(values)
    return result


class LaunchContract(unittest.TestCase):
    def setUp(self):
        self.Argument = type('DeclareLaunchArgument', (Action,), {})
        self.Node = type('Node', (Action,), {})
        self.Register = type('RegisterEventHandler', (Action,), {})
        self.Value = type('ParameterValue', (Action,), {})
        modules = {
            'ament_index_python': module('ament_index_python'),
            'ament_index_python.packages': module('ament_index_python.packages', get_package_share_directory=lambda _: '/installed/share/usb2can_demo_lingzu'),
            'launch': module('launch', LaunchDescription=LaunchDescription),
            'launch.actions': module('launch.actions', DeclareLaunchArgument=self.Argument, EmitEvent=Action, RegisterEventHandler=self.Register),
            'launch.event_handlers': module('launch.event_handlers', OnProcessExit=Action),
            'launch.events': module('launch.events', Shutdown=Action),
            'launch.substitutions': module('launch.substitutions', LaunchConfiguration=Action),
            'launch_ros': module('launch_ros'),
            'launch_ros.actions': module('launch_ros.actions', Node=self.Node),
            'launch_ros.parameter_descriptions': module('launch_ros.parameter_descriptions', ParameterValue=self.Value),
        }
        spec = importlib.util.spec_from_file_location('motor_launch', ROOT/'launch/usb2can_joystick.launch.py')
        launch_file = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(launch_file)
            self.entities = launch_file.generate_launch_description().entities
        self.arguments = {e.args[0]: e.kwargs.get('default_value') for e in self.entities if isinstance(e, self.Argument)}
        self.nodes = {e.kwargs['package']: e for e in self.entities if isinstance(e, self.Node)}

    def test_hardware_confirmation_default_and_type(self):
        self.assertEqual(self.arguments['hardware_confirmed'], 'false')
        value = self.nodes['usb2can_demo_lingzu'].kwargs['parameters'][1]['hardware_confirmed']
        self.assertIs(value.kwargs['value_type'], bool)

    def test_installed_config_location(self):
        self.assertTrue(self.arguments['motor_config'].replace('\\', '/').endswith('/config/motors.yaml'))
        params = self.nodes['usb2can_demo_lingzu'].kwargs['parameters']
        self.assertEqual(params[0].args[0], 'motor_config')
        self.assertEqual(params[1]['log_file'].args[0].args[0], 'log_file')

    def test_motion_mode_and_retiming_are_explicit(self):
        self.assertEqual(self.arguments['control_mode'], 'joystick')
        self.assertEqual(self.arguments['retime_trajectory'], 'false')
        params = self.nodes['usb2can_demo_lingzu'].kwargs['parameters'][1]
        self.assertIs(params['control_mode'].kwargs['value_type'], str)
        self.assertIs(params['retime_trajectory'].kwargs['value_type'], bool)

    def test_sdl_device_selection(self):
        params = self.nodes['joy'].kwargs['parameters'][0]
        self.assertIs(params['device_id'].kwargs['value_type'], int)
        self.assertIs(params['device_name'].kwargs['value_type'], str)
        self.assertEqual(self.nodes['joy'].kwargs['executable'], 'joy_node')
        self.assertNotIn('dev', params)

    def test_repeat_and_axis_deadzone(self):
        params = self.nodes['joy'].kwargs['parameters'][0]
        self.assertEqual(params['autorepeat_rate'], 20.0)
        self.assertEqual(params['deadzone'], 0.0)
        self.assertFalse(params['sticky_buttons'])

    def test_both_process_exits_shutdown_launch(self):
        handlers = [e.args[0] for e in self.entities if isinstance(e, self.Register)]
        self.assertEqual({id(h.kwargs['target_action']) for h in handlers}, {id(n) for n in self.nodes.values()})
        self.assertTrue(all(h.kwargs['on_exit'] for h in handlers))

    def test_handlers_registered_before_process_start(self):
        handlers = [i for i,e in enumerate(self.entities) if isinstance(e, self.Register)]
        nodes = [i for i,e in enumerate(self.entities) if isinstance(e, self.Node)]
        self.assertLess(max(handlers), min(nodes))


if __name__ == '__main__':
    unittest.main()
