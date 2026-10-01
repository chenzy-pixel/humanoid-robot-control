// Copyright (c) 2023-2025 TANGAIR
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "usb2can_transport.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <atomic>
#include <csignal>
#include <fstream>
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
        std::string model;
        lingzu::Limits limits;
        double lower, upper, neutral, direction, kp, kd;
        bool follow;
        lingzu::Command command;
        lingzu::Feedback feedback;
    };
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    std::vector<Joint> joints_;
    lingzu::Transport transport_;
    std::array<std::thread, 2> receivers_;
    std::mutex feedback_mutex_;
    std::atomic<bool> running_{true};
    bool enabled_ = false, hardware_confirmed_ = false, zero_requires_center_ = false;
    std::array<bool, 4> previous_buttons_{{false, false, false, false}};
    lingzu::InputWatchdog input_;
    lingzu::Clock::time_point enabled_since_{};
    const std::atomic<bool>* stop_signal_ = nullptr;
    int axis_ = 1;
    double deadzone_ = 0.15, axis_value_ = 0, input_timeout_ = 0.5, feedback_timeout_ = 0.5;
    uint8_t host_ = 0;
    std::ofstream log_;
    void loadConfig();
    void receive(unsigned device);
    void joystickCallback(sensor_msgs::msg::Joy::ConstSharedPtr message);
    bool broadcast(uint8_t type, bool flag = false, int attempts = 1);
    bool waitStopped(bool check_limits);
    void disable();
    void enable();
    void zero();
    void writeLog();
};
