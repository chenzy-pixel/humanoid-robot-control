# 人形机器人灵足电机控制

ROS 2 Jazzy 手柄控制工程，目标系统 Ubuntu 24.04，适用于经典 SOULDE USB2CAN 与 RS00、RS02、RS03、RS04 混用。每个设备/CAN 通道/电机 ID 独立配置型号、机械限位、方向与控制增益。

支持 [独立多关节轨迹控制](docs/多关节轨迹控制.md)：标准 `JointTrajectory` 输入、本地 Ruckig 插值、逐关节速度/加速度/jerk 限制，以及关节反馈和目标状态发布。配置必须填写各关节的 `max_velocity`、`max_acceleration` 和 `max_jerk`。

控制节点使用 `rclcpp`、`sensor_msgs/msg/Joy` 和 `ament_cmake`，通过 colcon 构建。USB2CAN 收发层直接编译：A8/A9 标记、小端 CAN ID、固定 17 字节串口包与 CRC8，支持分片、连续报文、短写、超时和端口排他锁。经典 CAN 速度为 1 Mbit/s，通道为 1/2。

| 目录 | 内容 |
| --- | --- |
| `ros2_ws/src/usb2can_demo_lingzu` | ROS 2 控制节点、ament 包、设备规则与参考 SDK |
| `common/include`、`common/src` | 电机协议、量程、发送调度、CRC 与 Linux 串口实现 |
| `config/motors.yaml` | ROS 2 原生参数，四型号混用示例 |
| `launch`、`scripts` | Python launch、colcon 构建和运行入口 |
| `tests` | 离线协议、IO、参数和启动检查 |
| `manuals` | 电机与 USB2CAN 协议手册 |
| `docs`、`dependencies` | 运行、配置和验证说明 |

安装 [依赖](dependencies/README.md)，从此目录执行：

```bash
bash scripts/build_ros2.sh
bash scripts/run_ros2.sh
```

先按实际装配修改 [电机配置](config/motors.yaml)。`motor_names` 列出启用的条目，`motors.<名称>` 包含各电机配置；删除未连接电机时同时移除其名称和配置。示例接线和 ±0.5 rad 范围需按实机填写。浮点字段包括 direction 使用 `1.0` 或 `-1.0`。

默认禁止使能、置零和清故障；配置核对后启动：

```bash
bash scripts/run_ros2.sh hardware_confirmed:=true
```

手柄使用 ROS 2 `joy_node`（SDL）。先运行 `ros2 run joy joy_enumerate_devices`，按编号或名称选择；通过 `ros2 topic echo /joy` 核对轴和按钮。轴 1 控制跟随电机，按钮 0 使能、1 失能、2 置零、3 清故障。SDL 的编号与旧 Linux 手柄接口可能不同，实际操作见 [运行指南](docs/运行指南.md)。

控制参数在启动时声明并设为只读；修改配置后重新启动。手柄消息采用 SensorData QoS，队列深度 1。控制与超时判断继续使用实际时间。输入/反馈超时、故障、状态异常、位置越界或发送失败触发失能。Ctrl+C/SIGTERM 请求退出，停止接收线程并尝试失能后关闭设备。

离线检查不访问电机或真实串口：

```bash
python3 scripts/check_repository.py
python3 -m unittest discover -s tests -p 'test_*.py'
cmake -S . -B build-check -DCMAKE_BUILD_TYPE=Debug
cmake --build build-check --parallel
ctest --test-dir build-check --output-on-failure
```

真实 ROS 2 参数测试在 Linux 上执行：

```bash
source ros2_ws/install/setup.bash
cd ros2_ws
colcon test --packages-select usb2can_demo_lingzu --return-code-on-test-failure
colcon test-result --verbose
```

[验证说明](docs/验证说明.md) 区分本机已完成的检查与尚需在目标机执行的集成测试。GitHub Actions 已配置 Ubuntu 离线检查与 Jazzy 容器中的真实 ament/rclcpp 构建、参数文件测试和已安装 launch 检查；Linux 离线检查、真实 Jazzy/colcon 构建、rclcpp 参数测试和安装入口检查均已通过 [GitHub CI](https://github.com/chenzy-pixel/humanoid-robot-control/actions/runs/36926500038)，记录见 [CI 证据](docs/validation/2026-10-02-github-ci.json)。

该 CI 记录对应轨迹功能加入前的基线。新增轨迹功能已接入离线测试和 CI 构建流程，真实 Jazzy 与实机验收范围见 [多关节轨迹控制](docs/多关节轨迹控制.md)。

源码：[电机协议](common/include/lingzu_protocol.hpp)、[调度与返回值](common/include/usb2can_transport.hpp)、[CRC 与串口解析](common/include/usb2can_packet.hpp)、[Linux 串口实现](common/src/usb_can.cpp)。[USB2CAN/USB2FDCAN 手册 v2.5](manuals/usb2can/SOULDE%20Studio%20USB2CAN及USB2FDCAN转换模块使用说明书v2.5.pdf) 中的经典 CAN 载荷为 0～8 字节；当前电机报文 DLC=8，使用固定 8 个数据槽位。

代码沿用 Apache-2.0 许可，来源见 [LICENSE](LICENSE) 和 [NOTICE](NOTICE)。上游：[SOULDE-Studio/USB2CAN-Demo-Lingzu](https://github.com/SOULDE-Studio/USB2CAN-Demo-Lingzu)。ROS 2 Jazzy 支持至 2029 年 5 月，见 [官方发行版表](https://github.com/ros2/ros2_documentation/blob/rolling/source/Releases.rst)。
