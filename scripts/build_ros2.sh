#!/usr/bin/env bash
set -eo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "$script_dir/.." && pwd)"
workspace="$project_root/ros2_ws"
ros_setup="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

[[ "$(uname -s)" == Linux ]] || { echo 'The USB2CAN transport requires Linux.' >&2; exit 1; }
[[ "${ROS_VERSION:-2}" == 2 ]] || { echo 'Use a fresh shell with a ROS 2 environment.' >&2; exit 1; }
[[ -r "$ros_setup" ]] || { echo "ROS 2 setup unavailable: $ros_setup. Set ROS_SETUP for a custom installation." >&2; exit 1; }
source "$ros_setup"
set -u
[[ "${ROS_VERSION:-}" == 2 ]] || { echo 'ROS_SETUP must point to a ROS 2 installation.' >&2; exit 1; }
command -v colcon >/dev/null 2>&1 || { echo 'Install python3-colcon-common-extensions.' >&2; exit 1; }
cd -- "$workspace"
colcon build --base-paths src --packages-select usb2can_demo_lingzu --cmake-args -DCMAKE_BUILD_TYPE=Release "$@"
echo 'Build complete. Load this workspace in your shell:'
printf 'source %q\n' "$workspace/install/setup.bash"
