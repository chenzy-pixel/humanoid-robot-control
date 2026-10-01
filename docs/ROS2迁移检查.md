# ROS 2 迁移检查（2026-10-02）

主工程原先使用 ROS 1 Noetic、Catkin、roscpp、XML launch 和 XmlRpc 电机列表。ROS 1 Noetic 已于 2025-05-31 结束官方维护，见 [官方公告](https://discourse.ros.org/t/ros-noetic-end-of-life-may-31-2025/43160)。用户目标机为 Ubuntu 24.04，因此改用 ROS 2 Jazzy（支持至 2029 年 5 月，见 [发行版表](https://github.com/ros2/ros2_documentation/blob/rolling/source/Releases.rst)）。

| 组件 | 当前实现 |
| --- | --- |
| 活动工作区 | `ros2_ws/src/usb2can_demo_lingzu` |
| 包构建与安装 | ament_cmake + colcon，C++17，安装节点、配置、launch 和 udev 规则 |
| 节点与手柄消息 | rclcpp、sensor_msgs/msg/Joy、单线程 executor |
| 参数 | 原生 ROS 2 参数，motor_names + motors.<name>，必需字段按类型检查并设为只读 |
| 手柄驱动 | ROS 2 joy_node，SDL 设备编号/名称，20 Hz 重发 |
| 启动与退出 | Python launch，进程退出联动，SIGINT/SIGTERM 请求有界停止 |
| 现有控制 | 保留型号量程、限位、停止反馈、失能与单调时钟超时判断 |
| 验证 | 离线协议/IO/参数/launch 检查，以及真实 Jazzy CI 配置 |

SOULDE USB2CAN SDK 和灵足报文层是独立硬件协议实现，不含 ROS 1 API，继续由 ROS 2 节点直接编译。Python 与独立 C++ 实验不依赖 ROS 运行时。历史 archive、原始参考资料及整理脚本保留当时的 ROS 1 路径，用于追溯；不进入当前构建。

迁移前活动源码快照保存在工作区根目录 `archive/reviews/2026-10-02-before-ros2`。更早的校验 JSON 保留当时的结果，当前结果另存到 `docs/validation/2026-10-02-ros2-checks.json`。

真实 ROS 2 和实机验证命令见 [运行指南](运行指南.md)。当前 Windows 工作区缺少 ROS 2 环境，WSL 无法访问；本机检查使用 API/IO 替代实现和 Linux 交叉编译。尚未验证 DDS 通信、实际 colcon 集成、手柄映射或电机硬件响应。
