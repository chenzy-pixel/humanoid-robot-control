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
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return descriptor;
}
double number(rclcpp::Node& node, const std::string& prefix, const char* key) {
    const double n = node.declare_parameter(prefix + key, rclcpp::ParameterType::PARAMETER_DOUBLE, read_only()).get_value<double>();
    if (!std::isfinite(n)) throw std::invalid_argument("Non-finite joint configuration");
    return n;
}
int integer(rclcpp::Node& node, const std::string& prefix, const char* key) {
    const auto n = node.declare_parameter(prefix + key, rclcpp::ParameterType::PARAMETER_INTEGER, read_only()).get_value<int64_t>();
    if (n < std::numeric_limits<int>::min() || n > std::numeric_limits<int>::max())
        throw std::invalid_argument("Integer joint field required");
    return static_cast<int>(n);
}
}
void Tangair_usb2can::loadConfig() {
    auto& config = *node_;
    hardware_confirmed_ = config.declare_parameter<bool>("hardware_confirmed", false, read_only());
    const auto axis = config.declare_parameter<int64_t>("joystick_axis", 1, read_only());
    deadzone_ = config.declare_parameter<double>("joystick_deadzone", 0.15, read_only());
    input_timeout_ = config.declare_parameter<double>("input_timeout", 0.5, read_only());
    feedback_timeout_ = config.declare_parameter<double>("feedback_timeout", 0.5, read_only());
    const auto host = config.declare_parameter<int64_t>("host_id", 0, read_only());
    if (host < 0 || host > 255 || axis < 0 || axis > std::numeric_limits<int>::max() ||
        !std::isfinite(deadzone_) || deadzone_ < 0 || deadzone_ >= 1 ||
        !std::isfinite(input_timeout_) || input_timeout_ <= 0 ||
        !std::isfinite(feedback_timeout_) || feedback_timeout_ <= 0)
        throw std::invalid_argument("Invalid host, joystick or timeout configuration");
    host_ = static_cast<uint8_t>(host);
    axis_ = static_cast<int>(axis);
    const auto names = config.declare_parameter<std::vector<std::string>>("motor_names", {}, read_only());
    if (names.empty()) throw std::invalid_argument("Configure nonempty motor_names and motors.<name> parameters");
    std::set<std::string> endpoints;
    std::set<std::string> unique_names;
    for (const auto& name : names) {
        if (name.empty() || name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_") != std::string::npos ||
            !unique_names.insert(name).second)
            throw std::invalid_argument("Motor names must be unique nonempty identifiers");
        const std::string prefix = "motors." + name + ".";
        Joint j{};
        int device = integer(config, prefix, "device"), channel = integer(config, prefix, "channel"), id = integer(config, prefix, "id");
        if (device < 0 || device > 1 || channel < 1 || channel > 2 || id < 1 || id > 254)
            throw std::invalid_argument("Invalid device/channel/motor ID");
        j.device = static_cast<unsigned>(device);
        j.channel = static_cast<uint8_t>(channel); j.id = static_cast<uint8_t>(id);
        j.model = config.declare_parameter(prefix + "model", rclcpp::ParameterType::PARAMETER_STRING, read_only()).get_value<std::string>();
        j.limits = lingzu::limits(j.model);
        j.lower = number(config, prefix, "min_position"); j.upper = number(config, prefix, "max_position");
        j.neutral = number(config, prefix, "neutral_position"); j.kp = number(config, prefix, "kp"); j.kd = number(config, prefix, "kd");
        j.direction = number(config, prefix, "direction");
        j.follow = config.declare_parameter(prefix + "follow_joystick", rclcpp::ParameterType::PARAMETER_BOOL, read_only()).get_value<bool>();
        if (j.lower >= j.upper || j.lower < -j.limits.position || j.upper > j.limits.position ||
            j.neutral < j.lower || j.neutral > j.upper || (j.direction != 1 && j.direction != -1))
            throw std::invalid_argument("Invalid mechanical limits, neutral position or direction");
        j.command.position = j.neutral; j.command.kp = j.kp; j.command.kd = j.kd;
        lingzu::control(j.id, j.limits, j.command); // Validate gains before opening hardware.
        std::string key = std::to_string(device) + ":" + std::to_string(channel) + ":" + std::to_string(id);
        if (!endpoints.insert(key).second) throw std::invalid_argument("Duplicate motor endpoint");
        RCLCPP_INFO(node_->get_logger(), "Joint %s model=%s mechanical range=[%.3f, %.3f]", key.c_str(), j.model.c_str(), j.lower, j.upper);
        joints_.push_back(j);
    }
}
Tangair_usb2can::Tangair_usb2can(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {
    if (!node_) throw std::invalid_argument("ROS 2 node is required");
    loadConfig();
    std::array<std::string, 2> paths{{"/dev/USB2CAN0", "/dev/USB2CAN1"}};
    paths[0] = node_->declare_parameter<std::string>("device0", paths[0], read_only());
    paths[1] = node_->declare_parameter<std::string>("device1", paths[1], read_only());
    const auto filename = node_->declare_parameter<std::string>("log_file", "motor_angle_log.csv", read_only());
    const std::string schema = "Timestamp_ms,Device,Channel,MotorID,Model,TargetAngleRad,ActualAngleRad,Fault,State,FeedbackValid,FeedbackAge_s";
    std::ifstream existing(filename);
    std::string previous_header;
    bool has_content = static_cast<bool>(std::getline(existing, previous_header));
    if (!previous_header.empty() && previous_header.back() == '\r') previous_header.pop_back();
    if (has_content && previous_header != schema)
        throw std::runtime_error("Existing motor log schema differs; choose a new log file/directory");
    log_.open(filename, std::ios::app);
    if (!log_) throw std::runtime_error("Cannot open motor log");
    if (!has_content) log_ << schema << '\n';
    if (!transport_.open(paths)) throw std::runtime_error("No USB2CAN device available");
    try {
        for (unsigned d = 0; d < 2; ++d)
            if (transport_.available(d)) receivers_[d] = std::thread(&Tangair_usb2can::receive, this, d);
        joy_sub_ = node_->create_subscription<sensor_msgs::msg::Joy>(
            "joy", rclcpp::SensorDataQoS().keep_last(1),
            [this](sensor_msgs::msg::Joy::ConstSharedPtr message) { joystickCallback(message); });
    } catch (...) {
        running_.store(false);
        for (auto& thread : receivers_) if (thread.joinable()) thread.join();
        throw;
    }
    if (!hardware_confirmed_) RCLCPP_WARN(node_->get_logger(), "Confirm motor wiring, models and limits in the config before enabling.");
}
Tangair_usb2can::~Tangair_usb2can() {
    joy_sub_.reset();
    running_.store(false);
    for (auto& thread : receivers_) if (thread.joinable()) thread.join();
    disable();
    transport_.close();
}
void Tangair_usb2can::receive(unsigned device) {
    while (running_.load()) {
        uint8_t channel = 0;
        lingzu::Frame frame;
        if (!transport_.receive(device, channel, frame)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1)); continue;
        }
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        for (auto& joint : joints_) {
            if (joint.device == device && joint.channel == channel && joint.id == ((frame.id >> 8) & 255))
                lingzu::feedback(frame, joint.limits, host_, joint.feedback);
        }
    }
}
bool Tangair_usb2can::broadcast(uint8_t type, bool flag, int attempts) {
    bool success = true;
    for (const auto& joint : joints_) {
        if (!transport_.available(joint.device)) { if (type == 3) success = false; continue; }
        if (!transport_.send(joint.device, joint.channel, lingzu::special(type, joint.id, host_, flag), attempts)) {
            RCLCPP_ERROR(node_->get_logger(), "CAN command %u failed: dev=%u channel=%u id=%u", type, joint.device, joint.channel, joint.id);
            success = false;
        }
    }
    return success;
}
void Tangair_usb2can::disable() {
    enabled_ = false;
    broadcast(4, false, 3);
}
bool Tangair_usb2can::waitStopped(bool check_limits) {
    const auto start = lingzu::Clock::now();
    enabled_ = false;
    if (!broadcast(4, false, 3)) return false;
    const double wait = std::min(0.25, std::min(input_timeout_, feedback_timeout_) / 2);
    while (std::chrono::duration<double>(lingzu::Clock::now() - start).count() < wait) {
        if (!running_.load() || !rclcpp::ok() || (stop_signal_ && stop_signal_->load(std::memory_order_relaxed))) return false;
        bool ready = true;
        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            for (const auto& joint : joints_) {
                const auto& f = joint.feedback;
                const double epsilon = 2 * joint.limits.position / 65535;
                if (!f.valid || f.received < start || f.state != 0 || f.fault ||
                    (check_limits && (f.position < joint.lower - epsilon || f.position > joint.upper + epsilon))) ready = false;
            }
        }
        if (ready) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    RCLCPP_ERROR(node_->get_logger(), "No fresh stopped feedback from every configured motor; operation refused.");
    return false;
}
void Tangair_usb2can::enable() {
    if (!hardware_confirmed_ || !input_.fresh(lingzu::Clock::now(), input_timeout_) ||
        (zero_requires_center_ && std::abs(axis_value_) > deadzone_)) {
        RCLCPP_WARN(node_->get_logger(), "Enable refused: check hardware confirmation, input freshness and centered joystick after zero.");
        return;
    }
    transport_.failed.store(false);
    if (!waitStopped(true)) return;
    for (const auto& joint : joints_)
        if (!transport_.send(joint.device, joint.channel, lingzu::run_mode(joint.id, host_, 0))) { disable(); return; }
    if (!broadcast(3)) { disable(); return; }
    enabled_ = true; zero_requires_center_ = false; enabled_since_ = lingzu::Clock::now();
}
void Tangair_usb2can::zero() {
    if (!hardware_confirmed_) return;
    for (const auto& joint : joints_) if (joint.lower > 0 || joint.upper < 0) {
        RCLCPP_WARN(node_->get_logger(), "Zero refused: configured mechanical range does not include zero"); return;
    }
    enabled_ = false;
    if (!waitStopped(false)) return;
    if (!broadcast(6, true)) return;
    for (auto& joint : joints_) joint.command.position = 0;
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        for (auto& joint : joints_) joint.feedback.valid = false;
    }
    zero_requires_center_ = true;
    RCLCPP_INFO(node_->get_logger(), "Mechanical zero requested. Center joystick and press enable to resume.");
}
void Tangair_usb2can::joystickCallback(sensor_msgs::msg::Joy::ConstSharedPtr message) {
    if (!running_.load()) return;
    if (message->axes.size() <= static_cast<unsigned>(axis_) || message->buttons.size() < 3 ||
        !std::isfinite(message->axes[axis_]) || std::abs(message->axes[axis_]) > 1.0001) {
        if (enabled_) disable();
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Invalid joystick message"); return;
    }
    axis_value_ = message->axes[axis_];
    input_.update(lingzu::Clock::now());
    double axis = std::abs(axis_value_) < deadzone_ ? 0 : std::max(-1.0, std::min(1.0, axis_value_));
    for (auto& joint : joints_) if (joint.follow)
        joint.command.position = (joint.lower + joint.upper) / 2 + joint.direction * axis * (joint.upper - joint.lower) / 2;
    std::array<bool, 4> pressed{{false, false, false, false}};
    for (unsigned i = 0; i < pressed.size() && i < message->buttons.size(); ++i) pressed[i] = message->buttons[i] == 1;
    // Stop has priority over enable when buttons are pressed together.
    if (pressed[1]) { if (!previous_buttons_[1]) disable(); }
    else if (pressed[2]) { if (!previous_buttons_[2]) zero(); }
    else if (pressed[3]) {
        if (!previous_buttons_[3] && hardware_confirmed_) { disable(); broadcast(4, true); }
    } else if (pressed[0] && !previous_buttons_[0]) enable();
    previous_buttons_ = pressed;
}
void Tangair_usb2can::writeLog() {
    auto now = lingzu::Clock::now();
    auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    for (const auto& joint : joints_) {
        const auto& f = joint.feedback;
        double age = f.valid ? std::chrono::duration<double>(now - f.received).count() : 0;
        bool valid = f.valid && age <= feedback_timeout_;
        log_ << timestamp << ',' << joint.device << ',' << int(joint.channel) << ',' << int(joint.id) << ','
             << joint.model << ',' << joint.command.position << ',';
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
    stop_signal_ = stop;
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_);
    rclcpp::WallRate rate(200);
    unsigned ticks = 0;
    while (running_.load() && rclcpp::ok() && (!stop || !stop->load(std::memory_order_relaxed))) {
        executor.spin_some(std::chrono::milliseconds(1));
        if (!running_.load() || !rclcpp::ok() || (stop && stop->load(std::memory_order_relaxed))) break;
        auto now = lingzu::Clock::now();
        bool stale = false;
        if (enabled_) {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            for (const auto& joint : joints_) {
                auto f = joint.feedback;
                if (f.valid && f.fault) stale = true;
                const double epsilon = 2 * joint.limits.position / 65535;
                if (f.valid && (f.position < joint.lower - epsilon || f.position > joint.upper + epsilon)) stale = true;
                if (std::chrono::duration<double>(now - enabled_since_).count() > feedback_timeout_ &&
                    (!f.valid || f.state != 2 || std::chrono::duration<double>(now - f.received).count() > feedback_timeout_)) stale = true;
            }
        }
        if (enabled_ && (!input_.fresh(now, input_timeout_) || stale || transport_.failed.load())) {
            RCLCPP_ERROR(node_->get_logger(), "Input/feedback timeout, motor fault or transport failure; disabling motors.");
            disable();
        }
        if (enabled_) {
            for (const auto& joint : joints_) {
                if (!transport_.send(joint.device, joint.channel, lingzu::control(joint.id, joint.limits, joint.command))) {
                    disable(); break;
                }
            }
        }
        writeLog();
        if (++ticks % 200 == 0) { log_.flush(); if (!log_) throw std::runtime_error("Motor log write failed"); }
        rate.sleep();
    }
    RequestStop();
}
