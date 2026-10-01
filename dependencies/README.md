# ROS 2 Jazzy 依赖

目标系统是 Ubuntu 24.04（x86-64 或 aarch64）及 ROS 2 Jazzy。先按 [官方安装说明](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) 安装 ROS 2，再安装本工程依赖：

```bash
sudo apt update
sudo apt install ros-jazzy-ros-base ros-jazzy-joy ros-jazzy-ament-cmake-gtest python3-colcon-common-extensions build-essential cmake
source /opt/ros/jazzy/setup.bash
for package in rclcpp rcl_interfaces sensor_msgs joy launch launch_ros ament_index_python; do ros2 pkg prefix "$package"; done
command -v colcon ros2
```

构建采用 ament_cmake、C++17 和 pthread，直接编译 `common/src/usb_can.cpp`。控制节点不需要预编译厂商 `.so`。`joy_node` 使用 SDL 的 event 输入接口，虚拟机需透传完整 USB 手柄并检查 `/dev/input/event*` 权限。

ROS 包清单见 `ros-packages.txt`，系统要求见 `system-requirements.txt`。Python 3 用于 launch 和离线检查，`ament_cmake_gtest` 用于真实 ROS 2 参数测试。
