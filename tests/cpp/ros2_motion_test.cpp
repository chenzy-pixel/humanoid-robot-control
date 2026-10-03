// Run the real controller loop against ROS API substitutes and simulated CAN feedback.
#include "Tangair_usb2can.h"
#include <cassert>
#include <iostream>
#include <map>
#include <mutex>

namespace {
struct Motor { double position, velocity = 0; uint8_t state = 0, fault = 0; bool frozen = false; };
std::mutex io_mutex;
std::map<uint8_t, Motor> motors;
std::vector<lingzu::Frame> controls;
bool fail_control = false, feedback_enabled = true;
bool overspeed = false;
int opens = 0, closes = 0, checks = 0;
std::vector<rclcpp::Parameter> config(const std::string& mode) {
    using P = rclcpp::Parameter;
    std::vector<P> values{P("motor_names", std::vector<std::string>{"a", "b", "c"}),
        P("hardware_confirmed", true), P("control_mode", mode), P("max_control_gap", 0.2),
        P("log_file", std::string("ros2_motion_test.csv"))};
    for (int i = 0; i < 3; ++i) {
        const std::string prefix = std::string("motors.") + char('a' + i) + ".";
        values.emplace_back(prefix + "device", int64_t{0}); values.emplace_back(prefix + "channel", int64_t{1});
        values.emplace_back(prefix + "id", int64_t{i + 1}); values.emplace_back(prefix + "model", std::string("RS00"));
        values.emplace_back(prefix + "min_position", -0.5); values.emplace_back(prefix + "max_position", 0.5);
        values.emplace_back(prefix + "neutral_position", 0.0); values.emplace_back(prefix + "direction", 1.0);
        values.emplace_back(prefix + "kp", 18.0); values.emplace_back(prefix + "kd", 0.8);
        values.emplace_back(prefix + "max_velocity", 0.5); values.emplace_back(prefix + "max_acceleration", 1.0);
        values.emplace_back(prefix + "max_jerk", 4.0); values.emplace_back(prefix + "follow_joystick", i == 0);
    }
    return values;
}
auto joy(int button = -1, float axis = 1) {
    auto message = std::make_shared<sensor_msgs::msg::Joy>();
    message->axes = {0, axis}; message->buttons = {0, 0, 0, 0};
    if (button >= 0) message->buttons[button] = 1;
    return message;
}
auto trajectory(double seconds = 2) {
    auto message = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    message->joint_names = {"b", "a"};
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {0.25, -0.2}; point.time_from_start.sec = static_cast<int32_t>(seconds);
    point.time_from_start.nanosec = static_cast<uint32_t>((seconds - point.time_from_start.sec) * 1e9);
    message->points.push_back(point); return message;
}
void resetIO() {
    std::lock_guard<std::mutex> lock(io_mutex);
    motors = {{1, {0.1}}, {2, {-0.1}}, {3, {0.15}}}; controls.clear();
    fail_control = false; feedback_enabled = true; overspeed = false;
    std::remove("ros2_motion_test.csv");
}
void run(const std::string& mode, const std::function<void(int, rclcpp::Node&)>& tick, int count) {
    resetIO(); rclcpp::initialized = true;
    rclcpp::NodeOptions options; auto node = std::make_shared<rclcpp::Node>("test", options.parameter_overrides(config(mode)));
    {
        Tangair_usb2can controller(node);
        node->dispatch(trajectory());
        { std::lock_guard<std::mutex> lock(io_mutex); assert(controls.empty()); ++checks; }
        node->dispatch(joy(0)); node->dispatch(joy());
        if (mode == "trajectory") node->dispatch(trajectory());
        int ticks = 0;
        node->on_spin = [&] { tick(ticks++, *node); if (ticks >= count) controller.RequestStop(); };
        controller.Spin();
        const auto commands = node->publisher<sensor_msgs::msg::JointState>("commanded_joint_states");
        assert(!commands->messages.empty() && commands->messages.front().name == std::vector<std::string>({"a", "b", "c"})); ++checks;
        const auto actual = node->publisher<sensor_msgs::msg::JointState>("joint_states");
        assert(actual->messages.size() == commands->messages.size()); ++checks;
    }
}
using Command = usb2can_demo_lingzu::srv::JointCommand;
using Action = control_msgs::action::FollowJointTrajectory;
using Handle = rclcpp_action::ServerGoalHandle<Action>;
auto command(rclcpp::Node& node, const std::string& operation, std::vector<std::string> names = {}, std::vector<double> positions = {}) {
    Command::Request request; request.operation = operation; request.joint_names = std::move(names); request.positions = std::move(positions);
    return node.interface<rclcpp::Service<Command>>("joint_command")->call(request);
}
auto action(rclcpp::Node& node) { return node.interface<rclcpp_action::Server<Action>>("follow_joint_trajectory"); }
Action::Goal goal(double target = 0.2) {
    Action::Goal value; value.trajectory.joint_names = {"a"};
    trajectory_msgs::msg::JointTrajectoryPoint point; point.positions = {target}; point.time_from_start.sec = 1;
    value.trajectory.points.push_back(point); return value;
}
void program(std::vector<rclcpp::Parameter> overrides, const std::function<void(rclcpp::Node&)>& setup,
             const std::function<void(int, rclcpp::Node&)>& tick, int count) {
    resetIO(); rclcpp::initialized = true; auto values = config("trajectory");
    values.emplace_back("input_source", std::string("program"));
    values.emplace_back("goal_settle_time", 0.03); values.emplace_back("goal_timeout", 0.3);
    values.emplace_back("calibration_file", std::string("program_test.calibration"));
    for (auto& replacement : overrides) {
        auto found = std::find_if(values.begin(), values.end(), [&](const auto& v) { return v.get_name() == replacement.get_name(); });
        if (found == values.end()) values.push_back(replacement); else *found = replacement;
    }
    rclcpp::NodeOptions options;
    auto node = std::make_shared<rclcpp::Node>("test", options.parameter_overrides(values));
    { Tangair_usb2can controller(node); setup(*node); int ticks = 0;
      node->on_spin = [&] { tick(ticks++, *node); if (ticks >= count) controller.RequestStop(); };
      controller.Spin();
    }
}
}
int32_t openUSBCAN(const char*) { return 42 + (opens++ % 2); }
int32_t closeUSBCAN(int32_t) { ++closes; return 0; }
int32_t sendUSBCAN(int32_t, uint8_t, FrameInfo* info, uint8_t* data) {
    std::lock_guard<std::mutex> lock(io_mutex);
    const auto type = (info->canID >> 24) & 31;
    auto& motor = motors.at(static_cast<uint8_t>(info->canID));
    if (type == 4) { motor.state = 0; motor.velocity = 0; }
    if (type == 3) motor.state = 2;
    if (type == 1) {
        if (fail_control) { fail_control = false; return -1; }
        lingzu::Frame frame; frame.id = info->canID; std::copy(data, data + 8, frame.data.begin()); controls.push_back(frame);
        const auto p = lingzu::limits("RS00");
        if (!motor.frozen) {
            motor.position = lingzu::decode(lingzu::get16(frame, 0), -p.position, p.position);
            motor.velocity = lingzu::decode(lingzu::get16(frame, 2), -p.velocity, p.velocity);
        }
    }
    return 17;
}
int32_t readUSBCAN(int32_t device, uint8_t* channel, FrameInfo* info, uint8_t* data, int32_t) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    std::lock_guard<std::mutex> lock(io_mutex);
    if (device != 42 || !feedback_enabled) return -1;
    static uint8_t next = 0; const uint8_t id = 1 + next++ % 3;
    const auto& motor = motors.at(id); const auto p = lingzu::limits("RS00");
    lingzu::Frame frame; frame.id = (2u << 24) | (uint32_t(motor.state) << 22) | (uint32_t(motor.fault) << 16) | (uint32_t(id) << 8);
    lingzu::put16(frame, 0, lingzu::encode(motor.position, -p.position, p.position));
    lingzu::put16(frame, 2, lingzu::encode(overspeed ? 0.6 : motor.velocity, -p.velocity, p.velocity));
    lingzu::put16(frame, 4, 32767); lingzu::put16(frame, 6, 250);
    info->canID = frame.id; info->frameType = EXTENDED; info->dataLength = 8; *channel = 1;
    std::copy(frame.data.begin(), frame.data.end(), data); return 0;
}
int main() {
    run("trajectory", [](int tick, rclcpp::Node& node) {
        node.dispatch(joy());
        if (tick == 50) { auto invalid = trajectory(); invalid->points[0].positions[0] = 10; node.dispatch(invalid); }
        if (tick == 51) { auto invalid = trajectory(); invalid->header.stamp.sec = 1; node.dispatch(invalid); }
        if (tick == 52) { auto invalid = trajectory(); invalid->points[0].effort = {0, 0}; node.dispatch(invalid); }
        if (tick == 420) {
            std::lock_guard<std::mutex> lock(io_mutex);
            assert(std::abs(motors.at(1).position + 0.2) < 0.001 && std::abs(motors.at(2).position - 0.25) < 0.001);
            assert(std::abs(motors.at(3).position - 0.15) < 0.001); ++checks;
            assert(node.publisher<std_msgs::msg::String>("trajectory_status")->messages.back().data == "completed"); ++checks;
        }
    }, 430);
    run("trajectory", [](int tick, rclcpp::Node& node) {
        node.dispatch(joy());
        if (tick == 50) node.dispatch(std::make_shared<trajectory_msgs::msg::JointTrajectory>());
        if (tick == 200) {
            assert(node.publisher<std_msgs::msg::String>("trajectory_status")->messages.back().data == "idle"); ++checks;
            std::lock_guard<std::mutex> lock(io_mutex);
            assert(motors.at(1).position > -0.2 && std::abs(motors.at(1).velocity) < 0.002); ++checks;
        }
    }, 210);
    run("trajectory", [](int tick, rclcpp::Node& node) {
        if (tick < 5) node.dispatch(joy());
        if (tick == 130) {
            assert(node.publisher<std_msgs::msg::String>("trajectory_status")->messages.back().data == "disabled"); ++checks;
            node.dispatch(joy(0)); node.dispatch(joy());
        }
        if (tick > 130) node.dispatch(joy());
        if (tick == 150) {
            assert(node.publisher<std_msgs::msg::String>("trajectory_status")->messages.back().data == "idle"); ++checks;
        }
    }, 155);
    run("trajectory", [](int tick, rclcpp::Node& node) {
        node.dispatch(joy());
        if (tick == 10) { std::lock_guard<std::mutex> lock(io_mutex); motors.at(1).fault = 1; }
        if (tick == 30) { assert(node.publisher<std_msgs::msg::String>("trajectory_status")->messages.back().data == "disabled"); ++checks; }
    }, 35);
    run("trajectory", [](int tick, rclcpp::Node& node) {
        node.dispatch(joy());
        if (tick == 10) { std::lock_guard<std::mutex> lock(io_mutex); fail_control = true; }
        if (tick == 20) { assert(node.publisher<std_msgs::msg::String>("trajectory_status")->messages.back().data == "disabled"); ++checks; }
    }, 25);
    run("joystick", [](int tick, rclcpp::Node& node) {
        node.dispatch(joy(-1, tick < 50 ? 1 : -1));
        if (tick == 80) {
            const auto& messages = node.publisher<sensor_msgs::msg::JointState>("commanded_joint_states")->messages;
            for (const auto& message : messages) assert(std::abs(message.velocity[0]) <= 0.5 + 1e-8);
            assert(messages.back().position[0] > -0.5 && messages.back().position[0] < 0.5); ++checks;
        }
    }, 85);
    run("trajectory", [](int tick, rclcpp::Node& node) {
        node.dispatch(joy());
        if (tick == 10) std::this_thread::sleep_for(std::chrono::milliseconds(250));
        if (tick == 20) { assert(node.publisher<std_msgs::msg::String>("trajectory_status")->messages.back().data == "disabled"); ++checks; }
    }, 25);
    run("trajectory", [](int tick, rclcpp::Node& node) {
        node.dispatch(joy());
        if (tick == 10) { std::lock_guard<std::mutex> lock(io_mutex); feedback_enabled = false; }
        if (tick == 130) {
            assert(node.publisher<std_msgs::msg::String>("trajectory_status")->messages.back().data == "disabled");
            assert(std::isnan(node.publisher<sensor_msgs::msg::JointState>("joint_states")->messages.back().position[0])); ++checks;
        }
    }, 135);
    run("trajectory", [](int tick, rclcpp::Node& node) {
        node.dispatch(joy());
        if (tick == 10) { std::lock_guard<std::mutex> lock(io_mutex); overspeed = true; }
        if (tick == 30) { assert(node.publisher<std_msgs::msg::String>("trajectory_status")->messages.back().data == "disabled"); ++checks; }
    }, 35);
    std::remove("program_test.calibration");
    std::shared_ptr<Handle> task;
    program({}, [&](rclcpp::Node& node) {
        assert(!command(node, "enable", {"a"}).success); ++checks;
        assert(command(node, "heartbeat").success); ++checks;
        assert(!command(node, "enable", {"a", "missing"}).success); ++checks;
        assert(command(node, "enable", {"a"}).success); ++checks;
        assert(!command(node, "calibrate", {"a"}, {0.0}).success); ++checks;
        auto state = command(node, "status"); assert(state.enabled == std::vector<bool>({true, false, false})); ++checks;
        auto invalid = goal(); invalid.goal_tolerance.push_back({"missing", 0.01, 0, 0});
        assert(!action(node)->send_goal(invalid)); ++checks;
        task = action(node)->send_goal(goal()); assert(task); ++checks;
    }, [&](int tick, rclcpp::Node& node) {
        command(node, "heartbeat");
        if (tick == 210) { assert(task->state == "succeeded" && task->result->error_code == 0 && !task->feedback.empty()); ++checks;
            assert(task->feedback.back().joint_names == std::vector<std::string>{"a"}); ++checks;
            assert(command(node, "enable", {"b"}).success); ++checks;
            assert(command(node, "disable", {"a"}).success); ++checks;
            assert(command(node, "status").enabled == std::vector<bool>({false, true, false})); ++checks;
        }
    }, 220);
    program({rclcpp::Parameter("motors.a.tracking_position_tolerance", 0.03), rclcpp::Parameter("tracking_error_timeout", 0.05)},
    [&](rclcpp::Node& node) {
        command(node, "heartbeat"); assert(command(node, "enable", {"a"}).success);
        { std::lock_guard<std::mutex> lock(io_mutex); motors.at(1).frozen = true; }
        task = action(node)->send_goal(goal(-0.2)); // too short: use a feasible 2s trajectory
        if (task && task->state == "aborted") { auto slower = goal(-0.2); slower.trajectory.points[0].time_from_start.sec = 2; task = action(node)->send_goal(slower); }
    }, [&](int tick, rclcpp::Node& node) {
        command(node, "heartbeat");
        if (tick == 180) { assert(task->state == "aborted" && task->result->error_code == Action::Result::PATH_TOLERANCE_VIOLATED); ++checks; }
    }, 190);
    program({}, [&](rclcpp::Node& node) {
        command(node, "heartbeat"); assert(command(node, "enable", {"a"}).success);
        { std::lock_guard<std::mutex> lock(io_mutex); motors.at(1).frozen = true; }
        task = action(node)->send_goal(goal());
    }, [&](int tick, rclcpp::Node& node) {
        command(node, "heartbeat");
        if (tick == 190) { assert(task->state == "executing"); ++checks; }
        if (tick == 280) { assert(task->state == "aborted" && task->result->error_code == Action::Result::GOAL_TOLERANCE_VIOLATED); ++checks;
            assert(node.publisher<usb2can_demo_lingzu::msg::TrajectoryTask>("trajectory_task")->messages.back().state == "failed"); ++checks;
        }
    }, 290);
    program({}, [&](rclcpp::Node& node) {
        command(node, "heartbeat"); assert(command(node, "enable", {"a"}).success);
        task = action(node)->send_goal(goal());
    }, [&](int tick, rclcpp::Node& node) {
        command(node, "heartbeat");
        if (tick == 30) { assert(action(node)->cancel_goal(task)); ++checks; }
        if (tick == 180) { assert(task->state == "canceled" && task->result->error_code == 0); ++checks; }
    }, 190);
    program({}, [&](rclcpp::Node& node) {
        command(node, "heartbeat"); command(node, "enable", {"a"}); task = action(node)->send_goal(goal());
    }, [&](int tick, rclcpp::Node&) {
        if (tick == 150) { assert(task->state == "aborted" && task->result->error_string.find("heartbeat") != std::string::npos); ++checks; }
    }, 160);
    program({rclcpp::Parameter("motors.a.direction", -1.0)}, [&](rclcpp::Node& node) {
        auto result = command(node, "calibrate", {"a"}, {0.2});
        assert(result.success && std::abs(result.positions[0] - 0.2) < 0.001 && std::abs(result.zero_offsets[0] - 0.3) < 0.001); ++checks;
        assert(!command(node, "calibrate", {"a"}, {10.0}).success); ++checks;
        command(node, "heartbeat"); assert(command(node, "enable", {"a"}).success);
        task = action(node)->send_goal(goal(0.1));
    }, [&](int tick, rclcpp::Node& node) {
        command(node, "heartbeat");
        if (tick == 240) { assert(task->state == "succeeded"); ++checks;
            std::lock_guard<std::mutex> lock(io_mutex); assert(std::abs(motors.at(1).position - 0.2) < 0.002); ++checks;
        }
    }, 250);
    program({rclcpp::Parameter("motors.a.direction", -1.0)}, [&](rclcpp::Node& node) {
        auto result = command(node, "status", {"a"}); assert(std::abs(result.zero_offsets[0] - 0.3) < 0.001); ++checks;
        result = command(node, "zero", {"a"}); assert(result.success && std::abs(result.positions[0]) < 0.001); ++checks;
    }, [](int, rclcpp::Node&) {}, 1);
    std::remove("program_test.calibration");
    program({rclcpp::Parameter("calibration_file", std::string("missing-directory/calibration"))}, [&](rclcpp::Node& node) {
        auto before = command(node, "status", {"a"}); auto result = command(node, "calibrate", {"a"}, {0.0});
        assert(!result.success && result.zero_offsets == before.zero_offsets); ++checks;
    }, [](int, rclcpp::Node&) {}, 1);
    assert(opens == closes); ++checks;
    std::remove("ros2_motion_test.csv");
    std::cout << checks << " ROS motion execution checks passed with simulated feedback\n";
}
