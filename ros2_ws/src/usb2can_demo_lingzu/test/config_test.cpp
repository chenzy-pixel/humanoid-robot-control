// Real rclcpp tests: every configuration fails before the transport is opened.
#include "Tangair_usb2can.h"
#include <gtest/gtest.h>
#include <limits>

class Ros2Config : public ::testing::Test {
protected:
    void SetUp() override { rclcpp::init(0, nullptr); }
    void TearDown() override { rclcpp::shutdown(); }
    std::vector<rclcpp::Parameter> joint() {
        using P = rclcpp::Parameter;
        return {P("motor_names", std::vector<std::string>{"joint"}),
            P("motors.joint.device", int64_t{0}), P("motors.joint.channel", int64_t{1}),
            P("motors.joint.id", int64_t{1}), P("motors.joint.model", "RS00"),
            P("motors.joint.min_position", -0.5), P("motors.joint.max_position", 0.5),
            P("motors.joint.neutral_position", 0.0), P("motors.joint.direction", 1.0),
            P("motors.joint.kp", 18.0), P("motors.joint.kd", 0.8),
            P("motors.joint.follow_joystick", true),
            P("motors.joint.max_velocity", 0.5), P("motors.joint.max_acceleration", 1.0), P("motors.joint.max_jerk", 4.0)};
    }
    rclcpp::Node::SharedPtr node(std::vector<rclcpp::Parameter> params) {
        rclcpp::NodeOptions options;
        return std::make_shared<rclcpp::Node>("can_motor_controller_node", options.parameter_overrides(params));
    }
};
TEST_F(Ros2Config, MissingMotorList) {
    auto n = node({}); EXPECT_THROW(Tangair_usb2can controller(n), std::invalid_argument);
    EXPECT_FALSE(n->get_parameter("hardware_confirmed").as_bool());
}
TEST_F(Ros2Config, RequiredModel) {
    auto params = joint(); params.erase(params.begin() + 4);
    auto n = node(params); EXPECT_THROW(Tangair_usb2can controller(n), std::exception);
}
TEST_F(Ros2Config, TypedConfirmation) {
    auto n = node({rclcpp::Parameter("hardware_confirmed", "true")});
    EXPECT_THROW(Tangair_usb2can controller(n), std::exception);
}
TEST_F(Ros2Config, ImmutableSafetyConfiguration) {
    auto n = node({}); EXPECT_THROW(Tangair_usb2can controller(n), std::invalid_argument);
    EXPECT_FALSE(n->set_parameter(rclcpp::Parameter("hardware_confirmed", true)).successful);
    EXPECT_FALSE(n->set_parameter(rclcpp::Parameter("feedback_timeout", 60.0)).successful);
}
TEST_F(Ros2Config, MechanicalLimits) {
    auto params = joint(); params[5] = rclcpp::Parameter("motors.joint.min_position", 0.6);
    auto n = node(params); EXPECT_THROW(Tangair_usb2can controller(n), std::invalid_argument);
}
TEST_F(Ros2Config, NonFiniteGain) {
    auto params = joint(); params[10] = rclcpp::Parameter("motors.joint.kd", std::numeric_limits<double>::quiet_NaN());
    auto n = node(params); EXPECT_THROW(Tangair_usb2can controller(n), std::invalid_argument);
}
TEST_F(Ros2Config, InvalidMotorId) {
    auto params = joint(); params[3] = rclcpp::Parameter("motors.joint.id", int64_t{255});
    auto n = node(params); EXPECT_THROW(Tangair_usb2can controller(n), std::invalid_argument);
}
TEST_F(Ros2Config, DuplicateNames) {
    auto params = joint(); params[0] = rclcpp::Parameter("motor_names", std::vector<std::string>{"joint", "joint"});
    auto n = node(params); EXPECT_THROW(Tangair_usb2can controller(n), std::invalid_argument);
}
TEST_F(Ros2Config, InvalidMotionLimits) {
    auto params = joint(); params[12] = rclcpp::Parameter("motors.joint.max_velocity", 0.0);
    auto n = node(params); EXPECT_THROW(Tangair_usb2can controller(n), std::invalid_argument);
}
TEST_F(Ros2Config, RequiredAccelerationLimit) {
    auto params = joint(); params.erase(params.begin() + 13);
    auto n = node(params); EXPECT_THROW(Tangair_usb2can controller(n), std::exception);
}
TEST_F(Ros2Config, InvalidControlMode) {
    auto params = joint(); params.emplace_back("control_mode", "unknown");
    auto n = node(params); EXPECT_THROW(Tangair_usb2can controller(n), std::invalid_argument);
}
TEST_F(Ros2Config, InvalidCoordinateOffset) {
    auto params = joint(); params.emplace_back("motors.joint.zero_offset", 20.0);
    auto n = node(params); EXPECT_THROW(Tangair_usb2can controller(n), std::invalid_argument);
}
TEST_F(Ros2Config, InvalidGoalTiming) {
    auto params = joint(); params.emplace_back("goal_timeout", 0.01);
    auto n = node(params); EXPECT_THROW(Tangair_usb2can controller(n), std::invalid_argument);
}
TEST_F(Ros2Config, InvalidTrackingTolerance) {
    auto params = joint(); params.emplace_back("motors.joint.tracking_position_tolerance", 0.0);
    auto n = node(params); EXPECT_THROW(Tangair_usb2can controller(n), std::invalid_argument);
}
TEST_F(Ros2Config, InvalidInputSource) {
    auto params = joint(); params.emplace_back("input_source", "unknown");
    auto n = node(params); EXPECT_THROW(Tangair_usb2can controller(n), std::invalid_argument);
}
TEST_F(Ros2Config, ExampleYamlUsesNativeRos2Parameters) {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "--params-file", MOTOR_CONFIG_FILE});
    // An empty log filename fails after config validation and before opening CAN.
    options.parameter_overrides({rclcpp::Parameter("log_file", "")});
    auto n = std::make_shared<rclcpp::Node>("can_motor_controller_node", options);
    try {
        Tangair_usb2can controller(n);
        FAIL() << "Empty log filename must fail before hardware access";
    } catch (const std::runtime_error& error) {
        EXPECT_STREQ(error.what(), "Cannot open motor log");
    }
    EXPECT_EQ(n->get_parameter("motor_names").as_string_array().size(), 12u);
    EXPECT_EQ(n->get_parameter("motors.motor_01.model").as_string(), "RS00");
    EXPECT_EQ(n->get_parameter("motors.motor_12.model").as_string(), "RS04");
    EXPECT_FALSE(n->get_parameter("hardware_confirmed").as_bool());
}
