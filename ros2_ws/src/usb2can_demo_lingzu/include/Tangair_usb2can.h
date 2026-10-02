// Copyright (c) 2023-2025 TANGAIR
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "usb2can_transport.hpp"
#include "joint_motion.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/string.hpp>
#include <atomic>
#include <csignal>
#include <fstream>
#include <memory>
#include <mutex>
#include <vector>

class Tangair_usb2can {
public:
    explicit Tangair_usb2can(rclcpp::Node::SharedPtr node);
    ~Tangair_usb2can();
    void Spin(const std::atomic<bool>* stop = nullptr);
    void RequestStop() { running_.store(false); }
private:
    struct Joint {
        unsigned device;
        uint8_t channel, id;
        std::string name, model;
        lingzu::Limits limits;
        double lower, upper, neutral, direction, kp, kd;
        double max_velocity, max_acceleration, max_jerk;
        bool follow;
        lingzu::Command command;
        lingzu::Feedback feedback;
    };
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_, command_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    std::vector<Joint> joints_;
    std::unique_ptr<lingzu::JointMotion> motion_;
    std::vector<double> joystick_targets_;
    std::string control_mode_, motion_status_ = "disabled";
    bool retime_trajectory_ = false;
    double max_control_gap_ = 0.05;
    lingzu::Transport transport_;
    std::array<std::thread, 2> receivers_;
    std::mutex feedback_mutex_;
    std::atomic<bool> running_{true};
    bool enabled_ = false, hardware_confirmed_ = false, zero_requires_center_ = false;
    std::array<bool, 4> previous_buttons_{{false, false, false, false}};
    lingzu::InputWatchdog input_;
    lingzu::Clock::time_point enabled_since_{};
    lingzu::Clock::time_point last_motion_tick_{};
    const std::atomic<bool>* stop_signal_ = nullptr;
    int axis_ = 1;
    double deadzone_ = 0.15, axis_value_ = 0, input_timeout_ = 0.5, feedback_timeout_ = 0.5;
    uint8_t host_ = 0;
    std::ofstream log_;
    void loadConfig();
    void receive(unsigned device);
    void joystickCallback(sensor_msgs::msg::Joy::ConstSharedPtr message);
    void trajectoryCallback(trajectory_msgs::msg::JointTrajectory::ConstSharedPtr message);
    void publishState();
    void updateMotion(lingzu::Clock::time_point now);
    bool broadcast(uint8_t type, bool flag = false, int attempts = 1);
    bool waitStopped(bool check_limits);
    void disable();
    void enable();
    void zero();
    void writeLog();
};
