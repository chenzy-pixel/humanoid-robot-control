// Copyright (c) 2023-2025 TANGAIR
// SPDX-License-Identifier: Apache-2.0
#include "Tangair_usb2can.h"
#include <algorithm>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <utility>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

namespace {
rcl_interfaces::msg::ParameterDescriptor read_only() {
    rcl_interfaces::msg::ParameterDescriptor value; value.read_only = true; return value;
}
double number(rclcpp::Node& node, const std::string& prefix, const char* key) {
    const double n = node.declare_parameter(prefix + key, rclcpp::ParameterType::PARAMETER_DOUBLE, read_only()).get<double>();
    if (!std::isfinite(n)) throw std::invalid_argument("Non-finite joint configuration");
    return n;
}
int integer(rclcpp::Node& node, const std::string& prefix, const char* key) {
    const auto n = node.declare_parameter(prefix + key, rclcpp::ParameterType::PARAMETER_INTEGER, read_only()).get<int64_t>();
    if (n < std::numeric_limits<int>::min() || n > std::numeric_limits<int>::max()) throw std::invalid_argument("Integer joint field required");
    return static_cast<int>(n);
}
double positive(rclcpp::Node& node, const std::string& name, double initial) {
    double n = node.declare_parameter<double>(name, initial, read_only());
    if (!std::isfinite(n) || n <= 0) throw std::invalid_argument("Positive finite parameter required: " + name);
    return n;
}
}
void Tangair_usb2can::loadConfig() {
    auto& config = *node_;
    hardware_confirmed_ = config.declare_parameter<bool>("hardware_confirmed", false, read_only());
    control_mode_ = config.declare_parameter<std::string>("control_mode", "joystick", read_only());
    input_source_ = config.declare_parameter<std::string>("input_source", "joystick", read_only());
    calibration_file_ = config.declare_parameter<std::string>("calibration_file", "joint_calibration.txt", read_only());
    tracking_timeout_ = positive(config, "tracking_error_timeout", 0.25);
    settle_time_ = positive(config, "goal_settle_time", 0.1);
    goal_wait_ = positive(config, "goal_timeout", 2.0);
    retime_trajectory_ = config.declare_parameter<bool>("retime_trajectory", false, read_only());
    max_control_gap_ = config.declare_parameter<double>("max_control_gap", 0.05, read_only());
    if ((control_mode_ != "joystick" && control_mode_ != "trajectory") ||
        (input_source_ != "joystick" && input_source_ != "program") || calibration_file_.empty() ||
        goal_wait_ < settle_time_ || !std::isfinite(max_control_gap_) || max_control_gap_ < 0.005 || max_control_gap_ > 0.5)
        throw std::invalid_argument("Invalid mode, calibration file or control/goal timing");
    const auto axis = config.declare_parameter<int64_t>("joystick_axis", 1, read_only());
    deadzone_ = config.declare_parameter<double>("joystick_deadzone", 0.15, read_only());
    input_timeout_ = positive(config, "input_timeout", 0.5);
    feedback_timeout_ = positive(config, "feedback_timeout", 0.5);
    const auto host = config.declare_parameter<int64_t>("host_id", 0, read_only());
    if (host < 0 || host > 255 || axis < 0 || axis > std::numeric_limits<int>::max() ||
        !std::isfinite(deadzone_) || deadzone_ < 0 || deadzone_ >= 1)
        throw std::invalid_argument("Invalid host or joystick configuration");
    host_ = static_cast<uint8_t>(host); axis_ = static_cast<int>(axis);
    const auto names = config.declare_parameter<std::vector<std::string>>("motor_names", {}, read_only());
    if (names.empty()) throw std::invalid_argument("Configure nonempty motor_names and motors.<name> parameters");
    std::set<std::string> endpoints, unique_names;
    for (const auto& name : names) {
        if (name.empty() || name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_") != std::string::npos ||
            !unique_names.insert(name).second) throw std::invalid_argument("Motor names must be unique nonempty identifiers");
        const std::string prefix = "motors." + name + ".";
        Joint j{}; j.name = name;
        int device = integer(config, prefix, "device"), channel = integer(config, prefix, "channel"), id = integer(config, prefix, "id");
        if (device < 0 || device > 1 || channel < 1 || channel > 2 || id < 1 || id > 254)
            throw std::invalid_argument("Invalid device/channel/motor ID");
        j.device = static_cast<unsigned>(device); j.channel = static_cast<uint8_t>(channel); j.id = static_cast<uint8_t>(id);
        j.model = config.declare_parameter(prefix + "model", rclcpp::ParameterType::PARAMETER_STRING, read_only()).get<std::string>();
        j.limits = lingzu::limits(j.model);
        j.lower = number(config, prefix, "min_position"); j.upper = number(config, prefix, "max_position");
        j.neutral = number(config, prefix, "neutral_position"); j.kp = number(config, prefix, "kp"); j.kd = number(config, prefix, "kd");
        j.direction = number(config, prefix, "direction");
        j.offset = config.declare_parameter<double>(prefix + "zero_offset", 0.0, read_only());
        j.max_velocity = number(config, prefix, "max_velocity");
        j.max_acceleration = number(config, prefix, "max_acceleration"); j.max_jerk = number(config, prefix, "max_jerk");
        j.path_position = positive(config, prefix + "tracking_position_tolerance", 0.15);
        j.path_velocity = positive(config, prefix + "tracking_velocity_tolerance", 0.5);
        j.goal_position = positive(config, prefix + "goal_position_tolerance", 0.01);
        j.goal_velocity = positive(config, prefix + "goal_velocity_tolerance", 0.02);
        j.follow = config.declare_parameter(prefix + "follow_joystick", rclcpp::ParameterType::PARAMETER_BOOL, read_only()).get<bool>();
        if (j.lower >= j.upper || j.neutral < j.lower || j.neutral > j.upper || (j.direction != 1 && j.direction != -1) ||
            j.max_velocity <= 0 || j.max_velocity > j.limits.velocity || j.max_acceleration <= 0 || j.max_jerk <= 0)
            throw std::invalid_argument("Invalid mechanical limits, neutral position, direction or motion limits");
        validateOffset(j, j.offset);
        j.command.position = j.neutral; j.command.kp = j.kp; j.command.kd = j.kd;
        lingzu::control(j.id, j.limits, motorCommand(j));
        std::string key = std::to_string(device) + ":" + std::to_string(channel) + ":" + std::to_string(id);
        if (!endpoints.insert(key).second) throw std::invalid_argument("Duplicate motor endpoint");
        joints_.push_back(j);
    }
    loadCalibration();
    std::vector<lingzu::JointMotionLimits> limits;
    lingzu::MotionState initial(joints_.size());
    for (std::size_t i = 0; i < joints_.size(); ++i) {
        const auto& j = joints_[i]; limits.push_back({j.name, j.lower, j.upper, j.max_velocity, j.max_acceleration, j.max_jerk});
        initial.position[i] = j.neutral;
    }
    motion_ = std::make_unique<lingzu::JointMotion>(std::move(limits)); motion_->reset(initial);
    joystick_targets_ = initial.position;
}
lingzu::Feedback Tangair_usb2can::jointFeedback(const Joint& j) const {
    auto f = j.feedback; f.position = j.direction * (f.position - j.offset);
    f.velocity *= j.direction; f.torque *= j.direction; return f;
}
lingzu::Command Tangair_usb2can::motorCommand(const Joint& j) const {
    auto c = j.command; c.position = j.offset + j.direction * c.position;
    c.velocity *= j.direction; c.torque *= j.direction; return c;
}
Tangair_usb2can::Indices Tangair_usb2can::select(const std::vector<std::string>& names) const {
    Indices indices; std::set<std::string> unique;
    if (names.empty()) { for (std::size_t i = 0; i < joints_.size(); ++i) indices.push_back(i); return indices; }
    for (const auto& name : names) {
        const auto found = std::find_if(joints_.begin(), joints_.end(), [&](const auto& j) { return j.name == name; });
        if (found == joints_.end() || !unique.insert(name).second) throw std::invalid_argument("Unknown or duplicate joint: " + name);
        indices.push_back(static_cast<std::size_t>(found - joints_.begin()));
    }
    return indices;
}
Tangair_usb2can::Tangair_usb2can(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {
    if (!node_) throw std::invalid_argument("ROS 2 node is required");
    loadConfig();
    std::array<std::string, 2> paths{{node_->declare_parameter<std::string>("device0", "/dev/USB2CAN0", read_only()),
                                    node_->declare_parameter<std::string>("device1", "/dev/USB2CAN1", read_only())}};
    const auto filename = node_->declare_parameter<std::string>("log_file", "motor_angle_log.csv", read_only());
    const std::string schema = "Timestamp_ms,Device,Channel,MotorID,Model,TargetAngleRad,ActualAngleRad,Fault,State,FeedbackValid,FeedbackAge_s";
    std::ifstream existing(filename); std::string previous;
    bool has_content = static_cast<bool>(std::getline(existing, previous));
    if (!previous.empty() && previous.back() == '\r') previous.pop_back();
    if (has_content && previous != schema) throw std::runtime_error("Existing motor log schema differs; choose a new log file/directory");
    log_.open(filename, std::ios::app);
    if (!log_) throw std::runtime_error("Cannot open motor log");
    if (!has_content) log_ << schema << '\n';
    if (!transport_.open(paths)) throw std::runtime_error("No USB2CAN device available");
    try {
        for (unsigned d = 0; d < 2; ++d) if (transport_.available(d)) receivers_[d] = std::thread(&Tangair_usb2can::receive, this, d);
        joy_sub_ = node_->create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::SensorDataQoS().keep_last(1),
            [this](sensor_msgs::msg::Joy::ConstSharedPtr m) { joystickCallback(m); });
        trajectory_sub_ = node_->create_subscription<trajectory_msgs::msg::JointTrajectory>("joint_trajectory", rclcpp::QoS(1),
            [this](trajectory_msgs::msg::JointTrajectory::ConstSharedPtr m) { trajectoryCallback(m); });
        state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());
        command_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("commanded_joint_states", rclcpp::SensorDataQoS());
        status_pub_ = node_->create_publisher<std_msgs::msg::String>("trajectory_status", rclcpp::QoS(1));
        createInterfaces();
    } catch (...) {
        running_.store(false); for (auto& thread : receivers_) if (thread.joinable()) thread.join(); throw;
    }
    if (!hardware_confirmed_) RCLCPP_WARN(node_->get_logger(), "Confirm motor wiring, models, coordinates and limits before enabling.");
}
Tangair_usb2can::~Tangair_usb2can() {
    joy_sub_.reset(); trajectory_sub_.reset(); command_service_.reset();
    running_.store(false); for (auto& thread : receivers_) if (thread.joinable()) thread.join();
    disable("Controller shutdown"); action_server_.reset(); transport_.close();
}
void Tangair_usb2can::receive(unsigned device) {
    while (running_.load()) {
        uint8_t channel = 0; lingzu::Frame frame;
        if (!transport_.receive(device, channel, frame)) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); continue; }
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        for (auto& j : joints_) if (j.device == device && j.channel == channel && j.id == ((frame.id >> 8) & 255))
            lingzu::feedback(frame, j.limits, host_, j.feedback);
    }
}
bool Tangair_usb2can::sendSpecial(const Indices& indices, uint8_t type, bool flag, int attempts) {
    bool success = true;
    for (auto i : indices) { const auto& j = joints_[i];
        if (!transport_.available(j.device) || !transport_.send(j.device, j.channel, lingzu::special(type, j.id, host_, flag), attempts)) success = false;
    }
    return success;
}
void Tangair_usb2can::disable(const std::string& reason, int code) {
    if (task_active_) finishTask("failed", reason, code);
    for (auto& j : joints_) { j.enabled = false; j.error_since = {}; j.command.velocity = 0; }
    enabled_ = false; motion_->clear(); motion_status_ = "disabled";
    sendSpecial(select({}), 4, false, 3);
}
bool Tangair_usb2can::waitStopped(const Indices& indices, bool check_limits) {
    const auto start = lingzu::Clock::now();
    if (!sendSpecial(indices, 4, false, 3)) return false;
    const double wait = std::min(0.25, std::min(input_timeout_, feedback_timeout_) / 2);
    while (std::chrono::duration<double>(lingzu::Clock::now() - start).count() < wait) {
        if (!running_.load() || !rclcpp::ok() || (stop_signal_ && stop_signal_->load())) return false;
        bool ready = true;
        { std::lock_guard<std::mutex> lock(feedback_mutex_);
          for (auto i : indices) {
              const auto& j = joints_[i]; auto f = jointFeedback(j);
              const double ep = 2 * j.limits.position / 65535, ev = 2 * j.limits.velocity / 65535;
              if (!f.valid || f.received < start || f.state != 0 || f.fault || std::abs(f.velocity) > ev ||
                  (check_limits && (f.position < j.lower - ep || f.position > j.upper + ep))) ready = false;
          }
        }
        if (ready) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
}
void Tangair_usb2can::enable(const Indices& selected) {
    if (!hardware_confirmed_ || !input_.fresh(lingzu::Clock::now(), input_timeout_) ||
        (zero_requires_center_ && input_source_ == "joystick" && std::abs(axis_value_) > deadzone_))
        throw std::runtime_error("Enable refused: hardware confirmation, fresh input or centered joystick required");
    Indices indices; for (auto i : selected) if (!joints_[i].enabled) indices.push_back(i);
    if (indices.empty()) return;
    if (motion_->active() || task_active_) throw std::runtime_error("Finish or cancel motion before enabling additional joints");
    transport_.failed.store(false);
    if (!waitStopped(indices, true)) throw std::runtime_error("No fresh stopped feedback from selected joints");
    auto start = motion_->state();
    { std::lock_guard<std::mutex> lock(feedback_mutex_);
      for (auto i : indices) { auto f = jointFeedback(joints_[i]); start.position[i] = std::clamp(f.position, joints_[i].lower, joints_[i].upper); start.velocity[i] = 0; start.acceleration[i] = 0; }
    }
    motion_->reset(start); joystick_targets_ = start.position;
    for (auto i : indices) { auto& j = joints_[i]; j.command.position = start.position[i]; j.command.velocity = 0;
        if (!transport_.send(j.device, j.channel, lingzu::run_mode(j.id, host_, 0))) { stopSelected(indices); throw std::runtime_error("Run mode write failed"); }
    }
    if (!sendSpecial(indices, 3)) { stopSelected(indices); throw std::runtime_error("Enable transmission failed"); }
    const auto now = lingzu::Clock::now();
    for (auto i : indices) { joints_[i].enabled = true; joints_[i].enabled_since = now; joints_[i].error_since = {}; }
    enabled_ = true; zero_requires_center_ = false; last_motion_tick_ = now; motion_status_ = "idle";
}
void Tangair_usb2can::stopSelected(const Indices& indices) {
    if (task_active_) finishTask("failed", "Joint operation interrupted the trajectory", Action::Result::INVALID_GOAL);
    for (auto i : indices) { joints_[i].enabled = false; joints_[i].command.velocity = 0; joints_[i].error_since = {}; }
    enabled_ = std::any_of(joints_.begin(), joints_.end(), [](const auto& j) { return j.enabled; });
    if (enabled_ && motion_->active()) {
        try { motion_->brake(); motion_status_ = "stopping"; }
        catch (const std::exception&) { disable("Cannot brake remaining joints"); }
    } else { motion_->clear(); motion_status_ = enabled_ ? "idle" : "disabled"; }
    if (!sendSpecial(indices, 4, false, 3)) throw std::runtime_error("Stop transmission failed for selected joints");
    last_motion_tick_ = lingzu::Clock::now();
}
void Tangair_usb2can::zero() {
    if (!hardware_confirmed_) return;
    try { stopSelected(select({})); operate("zero", select({}), {}); zero_requires_center_ = true; }
    catch (const std::exception& e) { RCLCPP_WARN(node_->get_logger(), "Zero refused: %s", e.what()); }
}
void Tangair_usb2can::joystickCallback(sensor_msgs::msg::Joy::ConstSharedPtr m) {
    if (!running_.load() || input_source_ != "joystick") return;
    if (m->axes.size() <= static_cast<unsigned>(axis_) || m->buttons.size() < 3 ||
        !std::isfinite(m->axes[axis_]) || std::abs(m->axes[axis_]) > 1.0001) {
        if (enabled_) disable("Invalid joystick message");
        return;
    }
    axis_value_ = m->axes[axis_]; input_.update(lingzu::Clock::now());
    double axis = std::abs(axis_value_) < deadzone_ ? 0 : std::clamp(axis_value_, -1.0, 1.0);
    if (control_mode_ == "joystick") {
        auto targets = joystick_targets_;
        for (std::size_t i = 0; i < joints_.size(); ++i) { const auto& j = joints_[i];
            if (j.follow && j.enabled) targets[i] = j.neutral + axis * (axis >= 0 ? j.upper - j.neutral : j.neutral - j.lower);
        }
        if (enabled_ && targets != joystick_targets_) {
            try { motion_->move(targets); motion_status_ = "running"; }
            catch (const std::exception& e) { disable(std::string("Joystick motion refused: ") + e.what()); }
        }
        joystick_targets_ = std::move(targets);
    }
    std::array<bool, 4> pressed{};
    for (unsigned i = 0; i < pressed.size() && i < m->buttons.size(); ++i) pressed[i] = m->buttons[i] == 1;
    try {
        if (pressed[1]) { if (!previous_buttons_[1]) disable(); }
        else if (pressed[2]) { if (!previous_buttons_[2]) zero(); }
        else if (pressed[3]) { if (!previous_buttons_[3] && hardware_confirmed_) operate("clear_fault", select({}), {}); }
        else if (pressed[0] && !previous_buttons_[0]) enable(select({}));
    } catch (const std::exception& e) { RCLCPP_WARN(node_->get_logger(), "Joystick operation refused: %s", e.what()); }
    previous_buttons_ = pressed;
}
void Tangair_usb2can::writeLog() {
    const auto now = lingzu::Clock::now();
    const auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    for (const auto& j : joints_) { auto f = jointFeedback(j); double age = f.valid ? std::chrono::duration<double>(now - f.received).count() : 0;
        bool valid = f.valid && age <= feedback_timeout_;
        log_ << timestamp << ',' << j.device << ',' << int(j.channel) << ',' << int(j.id) << ',' << j.model << ',' << j.command.position << ',';
        if (valid) log_ << f.position;
        log_ << ',';
        if (valid) log_ << int(f.fault);
        log_ << ',';
        if (valid) log_ << int(f.state);
        log_ << ',' << valid << ',';
        if (f.valid) log_ << age;
        log_ << '\n';
    }
}
void Tangair_usb2can::Spin(const std::atomic<bool>* stop) {
    stop_signal_ = stop; rclcpp::executors::SingleThreadedExecutor executor; executor.add_node(node_);
    rclcpp::WallRate rate(200); unsigned ticks = 0;
    while (running_.load() && rclcpp::ok() && (!stop || !stop->load())) {
        executor.spin_some(std::chrono::milliseconds(1));
        if (!running_.load() || !rclcpp::ok() || (stop && stop->load())) break;
        const auto now = lingzu::Clock::now(); std::string failure;
        if (enabled_) {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            for (const auto& j : joints_) if (j.enabled) {
                auto f = jointFeedback(j); const double ep = 2 * j.limits.position / 65535;
                if (f.valid && f.fault) failure = "Motor fault: " + j.name;
                else if (f.valid && (f.position < j.lower - ep || f.position > j.upper + ep)) failure = "Joint position outside limits: " + j.name;
                else if (f.valid && std::abs(f.velocity) > j.max_velocity + 2 * j.limits.velocity / 65535) failure = "Joint overspeed: " + j.name;
                else if (std::chrono::duration<double>(now - j.enabled_since).count() > feedback_timeout_ &&
                    (!f.valid || f.state != 2 || std::chrono::duration<double>(now - f.received).count() > feedback_timeout_)) failure = "Feedback timeout or invalid motor state: " + j.name;
                if (!failure.empty()) break;
            }
        }
        if (enabled_ && !input_.fresh(now, input_timeout_)) failure = "Input heartbeat timed out";
        if (enabled_ && transport_.failed.load()) failure = "USB2CAN transport failed";
        if (!failure.empty()) disable(failure);
        if (enabled_) updateMotion(now);
        if (enabled_) checkTracking(now);
        if (enabled_) for (const auto& j : joints_) if (j.enabled && !transport_.send(j.device, j.channel, lingzu::control(j.id, j.limits, motorCommand(j)))) {
            disable("Control transmission failed: " + j.name); break;
        }
        writeLog(); if (ticks % 4 == 0) publishState();
        if (++ticks % 200 == 0) { log_.flush(); if (!log_) throw std::runtime_error("Motor log write failed"); }
        rate.sleep();
    }
    RequestStop();
}
