#pragma once
#include <std_msgs/msg/header.hpp>
#include <string>
#include <vector>
namespace usb2can_demo_lingzu::msg {
struct TrajectoryTask {
    std_msgs::msg::Header header;
    std::string task_id, state, message;
    bool active = false;
    double progress = 0, elapsed_time = 0, planned_duration = 0;
    std::vector<std::string> joint_names;
    std::vector<double> desired_positions, actual_positions, position_errors;
};
}
