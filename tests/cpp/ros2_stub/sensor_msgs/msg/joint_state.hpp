#pragma once
#include <std_msgs/msg/header.hpp>
#include <string>
#include <vector>
namespace sensor_msgs::msg {
struct JointState {
    std_msgs::msg::Header header;
    std::vector<std::string> name;
    std::vector<double> position, velocity, effort;
};
}
