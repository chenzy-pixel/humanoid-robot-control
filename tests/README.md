# 代码检查

离线检查需要 CMake >= 3.16、C++17 编译器和 Python 3，不需要 ROS 或 USB2CAN：

```bash
python3 scripts/check_repository.py
python3 -m unittest discover -s tests -p 'test_*.py'
cmake -S . -B build-check -DCMAKE_BUILD_TYPE=Debug
cmake --build build-check --parallel
ctest --test-dir build-check --output-on-failure
```

`protocol_test` 包含 69 项协议和传输检查，`usb2can_io_test` 包含 23 项真实收发源码配合替代 Linux 系统调用的 IO 检查。

`ros2_config_test` 使用真实控制源码和 API/SDK 替代实现，验证配置在打开硬件前被检查、只读安全参数、未确认时禁止使能/置零/清故障，以及停止退出和设备关闭。`ros2_api_compile_check` 只检查两个节点源码的接口形状，Linux 下另编译实际串口系统接口。

`test_ros2_launch.py` 使用不启动进程的 launch 替代实现，检查默认禁使能、显式参数类型、安装路径、SDL 设备选择、输入重发和进程退出联动。

真实 rclcpp 测试位于 `ros2_ws/src/usb2can_demo_lingzu/test/config_test.cpp`，包含 9 项参数验证，并使用 ROS 2 参数解析器读取示例 YAML。在配置校验或日志打开阶段结束，不访问 CAN 硬件：

```bash
bash scripts/build_ros2.sh
source ros2_ws/install/setup.bash
cd ros2_ws
colcon test --packages-select usb2can_demo_lingzu --return-code-on-test-failure
colcon test-result --verbose
```

本机完成的范围见 [验证说明](../docs/验证说明.md)。真实 Jazzy 构建、rclcpp 参数测试及安装入口检查已通过 [GitHub CI](https://github.com/chenzy-pixel/humanoid-robot-control/actions/runs/36926500038)，记录见 [CI 证据](../docs/validation/2026-10-02-github-ci.json)。
