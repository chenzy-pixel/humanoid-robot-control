// Copyright (c) 2023-2025 TANGAIR
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "usb2can_transport.hpp"
#include "joint_motion.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <usb2can_demo_lingzu/srv/joint_command.hpp>
#include <usb2can_demo_lingzu/msg/trajectory_task.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/string.hpp>
#include <atomic>
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
    using Action = control_msgs::action::FollowJointTrajectory;
    using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
    using JointCommand = usb2can_demo_lingzu::srv::JointCommand;
    using Indices = std::vector<std::size_t>;
    struct Joint {
        unsigned device;
        uint8_t channel, id;
        std::string name, model;
        lingzu::Limits limits;
        double lower, upper, neutral, direction, offset, kp, kd;
        double max_velocity, max_acceleration, max_jerk;
        double path_position, path_velocity, goal_position, goal_velocity;
        bool follow, enabled = false;
        lingzu::Clock::time_point enabled_since{}, error_since{};
        lingzu::Command command; // joint coordinates
        lingzu::Feedback feedback; // motor coordinates, receiver mutex
    };
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_, command_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<usb2can_demo_lingzu::msg::TrajectoryTask>::SharedPtr task_pub_;
    rclcpp::Service<JointCommand>::SharedPtr command_service_;
    rclcpp_action::Server<Action>::SharedPtr action_server_;
    std::shared_ptr<GoalHandle> goal_;
    std::vector<Joint> joints_;
    std::unique_ptr<lingzu::JointMotion> motion_;
    std::vector<double> joystick_targets_;
    std::string control_mode_, input_source_, calibration_file_, motion_status_ = "disabled";
    std::string task_id_, task_state_ = "idle", task_message_;
    Indices task_joints_;
    std::vector<double> task_path_position_, task_path_velocity_, task_goal_position_, task_goal_velocity_;
    bool task_active_ = false, cancel_requested_ = false, retime_trajectory_ = false;
    uint64_t task_sequence_ = 0;
    double task_duration_ = 0, task_elapsed_ = 0, goal_wait_ = 2.0, task_goal_wait_ = 2.0;
    double max_control_gap_ = 0.05, tracking_timeout_ = 0.25, settle_time_ = 0.1;
    lingzu::Clock::time_point settling_since_{}, within_goal_since_{};
    lingzu::Transport transport_;
    std::array<std::thread, 2> receivers_;
    std::mutex feedback_mutex_;
    std::atomic<bool> running_{true};
    bool enabled_ = false, hardware_confirmed_ = false, zero_requires_center_ = false;
    std::array<bool, 4> previous_buttons_{{false, false, false, false}};
    lingzu::InputWatchdog input_;
    lingzu::Clock::time_point last_motion_tick_{};
    const std::atomic<bool>* stop_signal_ = nullptr;
    int axis_ = 1;
    double deadzone_ = 0.15, axis_value_ = 0, input_timeout_ = 0.5, feedback_timeout_ = 0.5;
    uint8_t host_ = 0;
    std::ofstream log_;
    void loadConfig();
    void loadCalibration();
    void saveCalibration(const std::vector<double>& offsets);
    void validateOffset(const Joint& joint, double offset) const;
    lingzu::Feedback jointFeedback(const Joint& joint) const;
    lingzu::Command motorCommand(const Joint& joint) const;
    Indices select(const std::vector<std::string>& names) const;
    bool sendSpecial(const Indices& indices, uint8_t type, bool flag = false, int attempts = 1);
    bool waitStopped(const Indices& indices, bool check_limits);
    void receive(unsigned device);
    void joystickCallback(sensor_msgs::msg::Joy::ConstSharedPtr message);
    void trajectoryCallback(trajectory_msgs::msg::JointTrajectory::ConstSharedPtr message);
    void createInterfaces();
    void commandCallback(const JointCommand::Request& request, JointCommand::Response& response);
    void operate(const std::string& operation, const Indices& indices, const std::vector<double>& positions);
    void enable(const Indices& indices);
    void disable(const std::string& reason = "Disabled by operator", int code = Action::Result::INVALID_GOAL);
    void zero();
    void stopSelected(const Indices& indices);
    std::vector<lingzu::MotionPoint> points(const trajectory_msgs::msg::JointTrajectory& trajectory) const;
    void validateGoal(const Action::Goal& goal) const;
    void startTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory, std::shared_ptr<GoalHandle> goal = nullptr);
    void cancelTask();
    void finishTask(const std::string& state, const std::string& reason, int code);
    void updateMotion(lingzu::Clock::time_point now);
    void checkTracking(lingzu::Clock::time_point now);
    void publishState();
    void writeLog();
};
