#pragma once
#include <std_msgs/msg/header.hpp>
#include <memory>
#include <string>
#include <vector>
namespace trajectory_msgs::msg {
struct JointTrajectoryPoint {
    std::vector<double> positions, velocities, accelerations, effort;
    builtin_interfaces::msg::Duration time_from_start;
};
struct JointTrajectory {
    using ConstSharedPtr = std::shared_ptr<const JointTrajectory>;
    std_msgs::msg::Header header;
    std::vector<std::string> joint_names;
    std::vector<JointTrajectoryPoint> points;
};
}
