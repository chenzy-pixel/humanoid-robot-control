// SPDX-License-Identifier: Apache-2.0
#include "Tangair_usb2can.h"
#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

void Tangair_usb2can::validateOffset(const Joint& j, double offset) const {
    if (!std::isfinite(offset) || std::abs(offset + j.direction * j.lower) > j.limits.position ||
        std::abs(offset + j.direction * j.upper) > j.limits.position)
        throw std::invalid_argument("Joint coordinate mapping exceeds motor protocol range: " + j.name);
}
void Tangair_usb2can::loadCalibration() {
    if (!std::filesystem::exists(calibration_file_)) return;
    std::ifstream input(calibration_file_); std::string schema;
    if (!std::getline(input, schema) || schema != "lingzu_joint_calibration_v1") throw std::runtime_error("Invalid calibration file header");
    std::string line; std::set<std::string> seen;
    while (std::getline(input, line)) {
        std::istringstream row(line); std::string name, model, extra; unsigned device, channel, id; double direction, offset;
        if (!(row >> name >> device >> channel >> id >> model >> direction >> offset) || row >> extra || !seen.insert(name).second)
            throw std::runtime_error("Invalid or duplicate calibration entry");
        const auto indices = select({name}); auto& j = joints_[indices[0]];
        if (j.device != device || j.channel != channel || j.id != id || j.model != model || j.direction != direction)
            throw std::runtime_error("Calibration hardware identity/direction mismatch: " + name);
        validateOffset(j, offset); j.offset = offset;
    }
    if (!input.eof()) throw std::runtime_error("Cannot read calibration file");
}
void Tangair_usb2can::saveCalibration(const std::vector<double>& offsets) {
    const auto temporary = calibration_file_ + ".tmp";
    std::ofstream output(temporary, std::ios::trunc);
    output << "lingzu_joint_calibration_v1\n" << std::setprecision(17);
    for (std::size_t i = 0; i < joints_.size(); ++i) { const auto& j = joints_[i];
        output << j.name << ' ' << j.device << ' ' << unsigned(j.channel) << ' ' << unsigned(j.id) << ' '
               << j.model << ' ' << j.direction << ' ' << offsets[i] << '\n';
    }
    output.flush(); if (!output) { output.close(); std::remove(temporary.c_str()); throw std::runtime_error("Cannot write calibration file"); }
    output.close(); if (!output) { std::remove(temporary.c_str()); throw std::runtime_error("Cannot close calibration file"); }
#ifdef _WIN32
    const bool committed = MoveFileExW(std::filesystem::path(temporary).c_str(), std::filesystem::path(calibration_file_).c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) != 0;
#else
    const bool committed = std::rename(temporary.c_str(), calibration_file_.c_str()) == 0;
#endif
    if (!committed) { std::remove(temporary.c_str()); throw std::runtime_error("Cannot replace calibration file"); }
}
void Tangair_usb2can::operate(const std::string& operation, const Indices& indices, const std::vector<double>& positions) {
    if (!running_.load()) throw std::runtime_error("Controller is stopping");
    if (operation != "calibrate" && !positions.empty()) throw std::invalid_argument("Positions are only accepted for calibrate");
    if (operation == "status") return;
    if (operation == "heartbeat") {
        if (input_source_ != "program") throw std::runtime_error("Select input_source=program for program heartbeat");
        input_.update(lingzu::Clock::now()); return;
    }
    if (operation == "disable") { stopSelected(indices); return; }
    if (!hardware_confirmed_) throw std::runtime_error("Hardware configuration is not confirmed");
    if (operation == "enable") { enable(indices); return; }
    if (operation == "clear_fault") {
        stopSelected(indices);
        if (!sendSpecial(indices, 4, true, 3)) throw std::runtime_error("Clear fault transmission failed");
        return;
    }
    if (operation != "calibrate" && operation != "zero") throw std::invalid_argument("Unknown joint operation");
    if (motion_->active() || task_active_) throw std::runtime_error("Finish or cancel motion before calibrating");
    for (auto i : indices) if (joints_[i].enabled) throw std::runtime_error("Disable selected joints before calibration");
    std::vector<double> known = operation == "zero" ? std::vector<double>(indices.size(), 0.0) : positions;
    if (known.size() != indices.size()) throw std::invalid_argument("One known position is required per selected joint");
    for (std::size_t n = 0; n < indices.size(); ++n) { const auto& j = joints_[indices[n]];
        if (!std::isfinite(known[n]) || known[n] < j.lower || known[n] > j.upper) throw std::invalid_argument("Known position outside joint limits: " + j.name);
    }
    if (!waitStopped(indices, false)) throw std::runtime_error("Calibration needs fresh stopped, fault-free feedback");
    std::vector<double> offsets; for (const auto& j : joints_) offsets.push_back(j.offset);
    { std::lock_guard<std::mutex> lock(feedback_mutex_);
      for (std::size_t n = 0; n < indices.size(); ++n) { const auto i = indices[n];
          offsets[i] = joints_[i].feedback.position - joints_[i].direction * known[n]; validateOffset(joints_[i], offsets[i]);
      }
    }
    // Persist first. A failed write leaves every effective coordinate unchanged.
    saveCalibration(offsets);
    auto state = motion_->state();
    for (std::size_t n = 0; n < indices.size(); ++n) { const auto i = indices[n]; auto& j = joints_[i];
        j.offset = offsets[i]; j.command.position = known[n]; j.command.velocity = 0;
        state.position[i] = known[n]; state.velocity[i] = 0; state.acceleration[i] = 0;
    }
    motion_->reset(state); joystick_targets_ = state.position; last_motion_tick_ = lingzu::Clock::now();
    zero_requires_center_ = true;
}
void Tangair_usb2can::commandCallback(const JointCommand::Request& request, JointCommand::Response& response) {
    Indices indices;
    try {
        indices = select(request.joint_names);
        if (request.operation == "heartbeat" && (!request.joint_names.empty() || !request.positions.empty()))
            throw std::invalid_argument("Heartbeat takes no joint names or positions");
        operate(request.operation, indices, request.positions);
        response.success = true;
        response.message = request.operation == "enable" ? "Enable command accepted; motor state remains monitored" : "Operation completed";
    } catch (const std::exception& error) { response.success = false; response.message = error.what(); }
    { std::lock_guard<std::mutex> lock(feedback_mutex_);
      const auto now = lingzu::Clock::now();
      for (auto i : indices) { const auto& j = joints_[i]; auto f = jointFeedback(j);
          response.joint_names.push_back(j.name); response.enabled.push_back(j.enabled);
          response.positions.push_back(f.valid && std::chrono::duration<double>(now - f.received).count() <= feedback_timeout_ ? f.position : std::numeric_limits<double>::quiet_NaN());
          response.directions.push_back(j.direction); response.zero_offsets.push_back(j.offset);
      }
    }
}
void Tangair_usb2can::validateGoal(const Action::Goal& goal) const {
    if (!goal.multi_dof_trajectory.joint_names.empty() || !goal.multi_dof_trajectory.points.empty() ||
        !goal.component_path_tolerance.empty() || !goal.component_goal_tolerance.empty())
        throw std::invalid_argument("Only scalar joints and position/velocity tolerances are supported");
    points(goal.trajectory);
    const auto indices = select(goal.trajectory.joint_names);
    for (auto i : indices) if (!joints_[i].enabled) throw std::invalid_argument("Trajectory joint is disabled: " + joints_[i].name);
    const auto check = [&](const auto& tolerances) {
        std::set<std::string> names;
        for (const auto& t : tolerances) {
            if (!names.insert(t.name).second || std::find(goal.trajectory.joint_names.begin(), goal.trajectory.joint_names.end(), t.name) == goal.trajectory.joint_names.end())
                throw std::invalid_argument("Unknown or duplicate tolerance joint");
            for (double value : {t.position, t.velocity, t.acceleration})
                if (!std::isfinite(value) || (value < 0 && value != -1)) throw std::invalid_argument("Invalid trajectory tolerance");
            if (t.acceleration > 0) throw std::invalid_argument("Measured acceleration tolerance is unavailable");
        }
    };
    check(goal.path_tolerance); check(goal.goal_tolerance);
    const auto& duration = goal.goal_time_tolerance;
    if (duration.sec < 0 || duration.nanosec >= 1000000000) throw std::invalid_argument("Invalid goal_time_tolerance");
    const double wait = duration.sec + duration.nanosec * 1e-9;
    if (wait != 0 && (wait < settle_time_ || wait > 3600)) throw std::invalid_argument("Goal time must cover settle time and be at most one hour");
}
void Tangair_usb2can::createInterfaces() {
    task_pub_ = node_->create_publisher<usb2can_demo_lingzu::msg::TrajectoryTask>("trajectory_task", rclcpp::QoS(1));
    command_service_ = node_->create_service<JointCommand>("joint_command",
        [this](const std::shared_ptr<JointCommand::Request> request, std::shared_ptr<JointCommand::Response> response) { commandCallback(*request, *response); });
    action_server_ = rclcpp_action::create_server<Action>(node_, "follow_joint_trajectory",
        [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const Action::Goal> goal) {
            if (!running_.load() || !enabled_ || control_mode_ != "trajectory" || cancel_requested_) return rclcpp_action::GoalResponse::REJECT;
            try { validateGoal(*goal); } catch (const std::exception& e) {
                RCLCPP_WARN(node_->get_logger(), "Action rejected: %s", e.what()); return rclcpp_action::GoalResponse::REJECT;
            }
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<GoalHandle> goal) {
            return goal == goal_ && task_active_ ? rclcpp_action::CancelResponse::ACCEPT : rclcpp_action::CancelResponse::REJECT;
        },
        [this](const std::shared_ptr<GoalHandle> goal) {
            try { startTrajectory(goal->get_goal()->trajectory, goal); }
            catch (const std::exception& e) { auto result = std::make_shared<Action::Result>(); result->error_code = Action::Result::INVALID_GOAL; result->error_string = e.what(); goal->abort(result); }
        });
}
