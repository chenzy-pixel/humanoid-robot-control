#!/usr/bin/env bash
set -eo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "$script_dir/.." && pwd)"
workspace_setup="$project_root/ros2_ws/install/setup.bash"
[[ "$(uname -s)" == Linux ]] || { echo 'The USB2CAN transport requires Linux.' >&2; exit 1; }
[[ "${ROS_VERSION:-2}" == 2 ]] || { echo 'Use a fresh shell with a ROS 2 environment.' >&2; exit 1; }
[[ -r "$workspace_setup" ]] || { echo "Build first: bash \"$script_dir/build_ros2.sh\"" >&2; exit 1; }
source "$workspace_setup"
set -u
[[ "${ROS_VERSION:-}" == 2 ]] || { echo 'The workspace must be built with ROS 2.' >&2; exit 1; }
command -v ros2 >/dev/null 2>&1 || { echo 'ros2 is unavailable in the loaded environment.' >&2; exit 1; }
export ROS_LOG_DIR="${ROS_LOG_DIR:-$project_root/logs/$(date +%Y%m%d_%H%M%S)_$$}"
mkdir -p -- "$ROS_LOG_DIR"
# Use the editable source config so hardware mapping changes do not require rebuilding.
exec ros2 launch usb2can_demo_lingzu usb2can_joystick.launch.py \
  "motor_config:=$project_root/config/motors.yaml" \
  "log_file:=$ROS_LOG_DIR/motor_angle_log.csv" "$@"
