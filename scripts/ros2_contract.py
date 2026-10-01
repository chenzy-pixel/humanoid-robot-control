"""Inspect ROS 2 package and launch contracts without importing hardware code."""
import ast
import xml.etree.ElementTree as ET


def check_ros2_contract(root):
    errors = []
    package_path = root / 'ros2_ws/src/usb2can_demo_lingzu/package.xml'
    launch_path = root / 'launch/usb2can_joystick.launch.py'
    try:
        package = ET.parse(package_path).getroot()
        if package.findtext('name') != 'usb2can_demo_lingzu' or package.get('format') != '3':
            errors.append('Expected ROS 2 package format 3 and usb2can_demo_lingzu name')
        if package.findtext('buildtool_depend') != 'ament_cmake' or package.findtext('export/build_type') != 'ament_cmake':
            errors.append('Expected ament_cmake build type')
        dependencies = {item.text for item in package if item.tag.endswith('depend')}
        if not {'rclcpp', 'rcl_interfaces', 'sensor_msgs', 'joy', 'launch', 'launch_ros', 'ament_index_python'} <= dependencies:
            errors.append('ROS 2 package is missing runtime dependencies')
        if {'catkin', 'roscpp', 'rospy'} & dependencies:
            errors.append('Active package still depends on ROS 1')
        tree = ast.parse(launch_path.read_text(encoding='utf-8'))
        arguments, nodes = {}, set()
        for call in ast.walk(tree):
            if not isinstance(call, ast.Call) or not isinstance(call.func, ast.Name):
                continue
            keywords = {item.arg: item.value for item in call.keywords}
            if call.func.id == 'DeclareLaunchArgument' and call.args and isinstance(call.args[0], ast.Constant):
                value = keywords.get('default_value')
                arguments[call.args[0].value] = value.value if isinstance(value, ast.Constant) else None
            if call.func.id == 'Node':
                values = [keywords.get(key) for key in ('package', 'executable')]
                if all(isinstance(value, ast.Constant) for value in values):
                    nodes.add(tuple(value.value for value in values))
        if arguments.get('hardware_confirmed') != 'false':
            errors.append('Launch must default hardware confirmation to false')
        if not {('joy', 'joy_node'), ('usb2can_demo_lingzu', 'can_node')} <= nodes:
            errors.append('Launch must start ROS 2 joystick and controller executables')
        if not {'motor_config', 'log_file', 'joystick_device_id', 'joystick_device_name'} <= arguments.keys():
            errors.append('Launch is missing configuration arguments')
    except (OSError, ET.ParseError, SyntaxError) as error:
        errors.append(str(error))
    return errors
