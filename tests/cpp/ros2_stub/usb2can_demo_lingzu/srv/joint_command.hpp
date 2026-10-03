#pragma once
#include <string>
#include <vector>
namespace usb2can_demo_lingzu::srv {
struct JointCommand {
    struct Request { std::string operation; std::vector<std::string> joint_names; std::vector<double> positions; };
    struct Response {
        bool success = false; std::string message;
        std::vector<std::string> joint_names;
        std::vector<bool> enabled;
        std::vector<double> positions, directions, zero_offsets;
    };
};
}
