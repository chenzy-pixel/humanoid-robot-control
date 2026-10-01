// Exercise real controller configuration with ROS 2 API and USB2CAN IO doubles.
#include "Tangair_usb2can.h"
#include <cassert>
#include <cstdio>
#include <iostream>
#include <limits>
#include <type_traits>
#include <utility>

static_assert(std::is_same<decltype(std::declval<rclcpp::Node&>().declare_parameter(
    std::declval<std::string>(), rclcpp::ParameterType::PARAMETER_DOUBLE)),
    const rclcpp::ParameterValue&>::value, "Typed declaration must match the Jazzy ParameterValue API");

namespace {
int opens = 0, closes = 0;
std::array<uint32_t, 64> sent_ids{};
std::size_t sent_count = 0;
std::vector<rclcpp::Parameter> config() {
    using P = rclcpp::Parameter;
    return {P("motor_names", std::vector<std::string>{"joint"}),
        P("motors.joint.device", int64_t{0}), P("motors.joint.channel", int64_t{1}),
        P("motors.joint.id", int64_t{1}), P("motors.joint.model", std::string("RS00")),
        P("motors.joint.min_position", -0.5), P("motors.joint.max_position", 0.5),
        P("motors.joint.neutral_position", 0.0), P("motors.joint.direction", 1.0),
        P("motors.joint.kp", 18.0), P("motors.joint.kd", 0.8),
        P("motors.joint.follow_joystick", true),
        P("log_file", std::string("ros2_config_test.csv"))};
}
void replace(std::vector<rclcpp::Parameter>& values, rclcpp::Parameter value) {
    for (auto& current : values) if (current.get_name() == value.get_name()) { current = value; return; }
    values.push_back(value);
}
void rejected(const std::vector<rclcpp::Parameter>& values) {
    rclcpp::NodeOptions options;
    auto node = std::make_shared<rclcpp::Node>("can_motor_controller_node", options.parameter_overrides(values));
    bool failed = false;
    const int before = opens;
    try { Tangair_usb2can controller(node); } catch (const std::exception&) { failed = true; }
    assert(failed && opens == before); // Invalid config must fail before any hardware IO.
}
}
int32_t openUSBCAN(const char*) { return 42 + opens++; }
int32_t closeUSBCAN(int32_t) { ++closes; return 0; }
int32_t sendUSBCAN(int32_t, uint8_t, FrameInfo* info, uint8_t*) { sent_ids.at(sent_count++) = info->canID; return 17; }
int32_t readUSBCAN(int32_t, uint8_t*, FrameInfo*, uint8_t*, int32_t) { return -1; }

int main() {
    using P = rclcpp::Parameter;
    int checks = 0;
    rejected({}); ++checks;
    auto values = config(); values.erase(values.begin() + 4); rejected(values); ++checks;
    const std::vector<P> invalid{
        P("motor_names", std::vector<std::string>{"joint", "joint"}),
        P("motor_names", std::vector<std::string>{"invalid.name"}),
        P("motors.joint.id", int64_t{255}), P("motors.joint.channel", int64_t{3}),
        P("motors.joint.model", std::string("unknown")),
        P("motors.joint.min_position", 0.6), P("motors.joint.neutral_position", 0.6),
        P("motors.joint.direction", 0.0), P("motors.joint.kp", -1.0),
        P("motors.joint.kd", std::numeric_limits<double>::quiet_NaN()),
        P("input_timeout", 0.0), P("feedback_timeout", -1.0),
        P("joystick_deadzone", 1.0), P("joystick_axis", int64_t{-1}),
        P("joystick_axis", std::numeric_limits<int64_t>::max()),
        P("host_id", int64_t{256}), P("hardware_confirmed", std::string("true")),
        P("motors.joint.direction", int64_t{1}),
    };
    for (const auto& invalid_value : invalid) {
        values = config(); replace(values, invalid_value); rejected(values); ++checks;
    }
    values = config();
    replace(values, P("motor_names", std::vector<std::string>{"joint", "duplicate"}));
    const auto first = config();
    for (const auto& value : first) if (value.get_name().find("motors.joint.") == 0) {
        const auto name = "motors.duplicate." + value.get_name().substr(13);
        if (value.get_name().find(".model") != std::string::npos) values.emplace_back(name, value.get_value<std::string>());
        else if (value.get_name().find(".follow_joystick") != std::string::npos) values.emplace_back(name, value.get_value<bool>());
        else if (value.get_name().find(".device") != std::string::npos || value.get_name().find(".channel") != std::string::npos || value.get_name().find(".id") != std::string::npos)
            values.emplace_back(name, value.get_value<int64_t>());
        else values.emplace_back(name, value.get_value<double>());
    }
    rejected(values); ++checks;
    std::remove("ros2_config_test.csv");
    rclcpp::NodeOptions options;
    auto node = std::make_shared<rclcpp::Node>("can_motor_controller_node", options.parameter_overrides(config()));
    {
        Tangair_usb2can controller(node);
        assert(opens == 2 && sent_count == 0 && node->subscription_depth == 1); ++checks;
        assert(!node->set_parameter(P("hardware_confirmed", true)).successful); ++checks;
        assert(!node->set_parameter(P("motors.joint.max_position", 20.0)).successful); ++checks;
        auto joy = std::make_shared<sensor_msgs::msg::Joy>();
        joy->axes = {0.0f, 0.0f}; joy->buttons = {1, 0, 0, 0}; node->dispatch(joy);
        assert(sent_count == 0); ++checks; // Default hardware confirmation blocks enabling.
        joy->buttons = {0, 0, 1, 0}; node->dispatch(joy); assert(sent_count == 0); ++checks;
        joy->buttons = {0, 0, 0, 1}; node->dispatch(joy); assert(sent_count == 0); ++checks;
        std::atomic<bool> stop{true}; controller.Spin(&stop);
        assert(sent_count == 0); ++checks;
    }
    assert(closes == 2 && sent_count == 1 && ((sent_ids[0] >> 24) & 31) == 4); ++checks;
    std::remove("ros2_config_test.csv");
    std::cout << checks << " ROS 2 configuration and shutdown checks passed with API/IO doubles\n";
}
