# 代码检查

离线检查需要 CMake >= 3.16、C++17 编译器和 Python 3，不需要 ROS 或 USB2CAN：

Python 单元测试包含 8 项 launch 构造检查和 17 项 USB2CAN CRC、帧解析、通道隔离与 SDK 接口检查。

```bash
python3 scripts/check_repository.py
python3 -m unittest discover -s tests -p 'test_*.py'
cmake -S . -B build-check -DCMAKE_BUILD_TYPE=Debug
cmake --build build-check --parallel
ctest --test-dir build-check --output-on-failure
```

`protocol_test` 包含 69 项协议和传输检查，`usb2can_io_test` 包含 23 项真实收发源码配合替代 Linux 系统调用的 IO 检查。

`trajectory_test` 使用真实 Ruckig 和运动规划源码，检查独立关节、同步、密集采样下的运动限制、轨迹替换、重定时、曲线越界和减速取消。`ros2_motion_test` 运行真实控制循环，使用模拟 CAN 反馈验证下发、状态、输入超时、故障、发送失败和重新使能，不打开实际串口。

`ros2_config_test` 使用真实控制源码和 API/SDK 替代实现，验证配置在打开硬件前被检查、只读安全参数、未确认时禁止使能/置零/清故障，以及停止退出和设备关闭。`ros2_api_compile_check` 检查入口、控制循环、程序接口和轨迹状态四个源文件的接口形状，Linux 下另编译实际串口系统接口。

闭环与程序接口回归还覆盖实际不到位、持续跟踪偏差、程序 heartbeat 丢失、Action 接受/取消/替换与终态、逐关节操作、统一方向/偏置、标定保存和加载以及保存失败的状态保留。配置测试包含偏置范围、跟踪容差与终点等待时间校验。

真实服务/Action 测试运行生产 `can_node`、rclpy 客户端和虚拟 PTY 电机，使用真实 DDS 与串口系统调用：

```bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 tests/ros2/test_program_interface.py
```

CI 的 Jazzy 集成任务配置了这一测试脚本，测试使用虚拟串口。

`test_ros2_launch.py` 使用不启动进程的 launch 替代实现，检查默认禁使能、显式参数类型、安装路径、SDL 设备选择、输入重发和进程退出联动。

真实 rclcpp 测试位于 `ros2_ws/src/usb2can_demo_lingzu/test/config_test.cpp`，包含 16 项参数验证，并使用 ROS 2 参数解析器读取示例 YAML。在配置校验或日志打开阶段结束，不访问 CAN 硬件：

```bash
bash scripts/build_ros2.sh
source ros2_ws/install/setup.bash
cd ros2_ws
colcon test --packages-select usb2can_demo_lingzu --return-code-on-test-failure
colcon test-result --verbose
```

本机完成的范围见 [验证说明](../docs/验证说明.md)。真实 Jazzy 构建、rclcpp 参数测试及安装入口检查已通过 [GitHub CI](https://github.com/chenzy-pixel/humanoid-robot-control/actions/runs/36926500038)，记录见 [CI 证据](../docs/validation/2026-10-02-github-ci.json)。

上述 CI 记录对应轨迹功能加入前的源码基线。当前轨迹功能的离线测试与目标 Linux 验收范围见 [轨迹说明](../docs/多关节轨迹控制.md)。
