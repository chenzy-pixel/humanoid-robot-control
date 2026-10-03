// SPDX-License-Identifier: Apache-2.0
#include "Tangair_usb2can.h"
#include <algorithm>
#include <iomanip>
#include <limits>
#include <sstream>

std::vector<lingzu::MotionPoint> Tangair_usb2can::points(const trajectory_msgs::msg::JointTrajectory& t) const {
    if (t.header.stamp.sec != 0 || t.header.stamp.nanosec != 0) throw std::invalid_argument("Use a zero header stamp for immediate execution");
    if (t.joint_names.empty() || t.points.empty() || t.points.size() > 100) throw std::invalid_argument("Use named joints and 1..100 points");
    select(t.joint_names);
    std::vector<lingzu::MotionPoint> converted;
    for (const auto& p : t.points) {
        if (p.time_from_start.sec < 0 || p.time_from_start.nanosec >= 1000000000 || !p.effort.empty())
            throw std::invalid_argument("Use nonnegative durations and position/velocity/acceleration fields");
        converted.push_back({p.time_from_start.sec + p.time_from_start.nanosec * 1e-9, p.positions, p.velocities, p.accelerations});
    }
    return converted;
}
void Tangair_usb2can::startTrajectory(const trajectory_msgs::msg::JointTrajectory& t, std::shared_ptr<GoalHandle> goal) {
    if (!running_.load() || control_mode_ != "trajectory" || !enabled_ || cancel_requested_)
        throw std::runtime_error("Select trajectory mode, enable joints and complete cancellation first");
    const auto selected = select(t.joint_names);
    for (auto i : selected) if (!joints_[i].enabled) throw std::invalid_argument("Trajectory joint is disabled: " + joints_[i].name);
    auto converted = points(t);
    if (goal) validateGoal(*goal->get_goal());
    // JointMotion commits atomically after validation. Rejected plans preserve the old task.
    const double duration = motion_->plan(t.joint_names, converted, retime_trajectory_);
    if (task_active_) finishTask("failed", "Replaced by a new trajectory", Action::Result::INVALID_GOAL);
    goal_ = std::move(goal); task_joints_ = selected; task_duration_ = duration; task_elapsed_ = 0; task_goal_wait_ = goal_wait_;
    task_path_position_.clear(); task_path_velocity_.clear(); task_goal_position_.clear(); task_goal_velocity_.clear();
    for (const auto& j : joints_) {
        task_path_position_.push_back(j.path_position); task_path_velocity_.push_back(j.path_velocity);
        task_goal_position_.push_back(j.goal_position); task_goal_velocity_.push_back(j.goal_velocity);
    }
    if (goal_) {
        std::ostringstream id; id << std::hex << std::setfill('0');
        for (auto b : goal_->get_goal_id()) id << std::setw(2) << unsigned(b);
        task_id_ = id.str();
        const auto& requested = *goal_->get_goal();
        auto apply = [&](const auto& tolerances, auto& position, auto& velocity) {
            for (const auto& t1 : tolerances) { const auto i = select({t1.name})[0];
                if (t1.position > 0) position[i] = std::min(position[i], t1.position);
                if (t1.velocity > 0) velocity[i] = std::min(velocity[i], t1.velocity);
            }
        };
        apply(requested.path_tolerance, task_path_position_, task_path_velocity_);
        apply(requested.goal_tolerance, task_goal_position_, task_goal_velocity_);
        double wait = requested.goal_time_tolerance.sec + requested.goal_time_tolerance.nanosec * 1e-9;
        if (wait > 0) task_goal_wait_ = wait;
    } else task_id_ = "topic-" + std::to_string(++task_sequence_);
    task_active_ = true; cancel_requested_ = false; task_message_ = "Trajectory accepted";
    settling_since_ = {}; within_goal_since_ = {};
    motion_status_ = task_state_ = motion_->active() ? "running" : "settling";
    if (!motion_->active()) settling_since_ = lingzu::Clock::now();
}
void Tangair_usb2can::finishTask(const std::string& state, const std::string& reason, int code) {
    if (!task_active_) return;
    task_elapsed_ = motion_->elapsed();
    task_state_ = state; task_message_ = reason; task_active_ = false; cancel_requested_ = false;
    if (goal_) {
        auto result = std::make_shared<Action::Result>(); result->error_code = code; result->error_string = reason;
        try {
            if (goal_->is_active()) {
                if (state == "completed") goal_->succeed(result);
                else if (state == "canceled" && goal_->is_canceling()) goal_->canceled(result);
                else goal_->abort(result);
            }
        } catch (const std::exception& e) { RCLCPP_ERROR(node_->get_logger(), "Cannot publish terminal action result: %s", e.what()); }
        goal_.reset();
    }
}
void Tangair_usb2can::cancelTask() {
    if (!task_active_) return;
    motion_->brake(); cancel_requested_ = true; task_state_ = motion_status_ = "stopping";
    task_duration_ = motion_->duration();
    task_message_ = "Cancellation requested; waiting for measured stop";
    settling_since_ = {}; within_goal_since_ = {}; task_goal_wait_ = goal_wait_;
}
void Tangair_usb2can::trajectoryCallback(trajectory_msgs::msg::JointTrajectory::ConstSharedPtr message) {
    bool cancel_attempted = false;
    try {
        if (message->points.empty()) {
            if (!message->joint_names.empty() || message->header.stamp.sec != 0 || message->header.stamp.nanosec != 0)
                throw std::invalid_argument("Cancel with an empty zero-stamped trajectory");
            if (task_active_) { cancel_attempted = true; cancelTask(); }
            return;
        }
        startTrajectory(*message);
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "Trajectory refused: %s", e.what());
        if (cancel_attempted && task_active_) disable(std::string("Cancellation failed: ") + e.what());
    }
}
void Tangair_usb2can::updateMotion(lingzu::Clock::time_point now) {
    const double dt = std::chrono::duration<double>(now - last_motion_tick_).count(); last_motion_tick_ = now;
    if (dt < 0 || dt > max_control_gap_) { disable("Control cycle missed its motion deadline"); return; }
    try {
        if (goal_ && goal_->is_canceling() && !cancel_requested_) cancelTask();
        if (motion_->advance(dt)) {
            if (task_active_) { motion_status_ = task_state_ = "settling"; settling_since_ = now; within_goal_since_ = {}; }
            else motion_status_ = "idle";
        }
        const auto& state = motion_->state();
        for (std::size_t i = 0; i < joints_.size(); ++i) if (joints_[i].enabled) {
            joints_[i].command.position = state.position[i]; joints_[i].command.velocity = state.velocity[i];
        }
    } catch (const std::exception& e) { disable(std::string("Motion execution failed: ") + e.what()); }
}
void Tangair_usb2can::checkTracking(lingzu::Clock::time_point now) {
    std::string failure; bool arrived = task_active_ && !motion_->active();
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        for (std::size_t i = 0; i < joints_.size(); ++i) { auto& j = joints_[i]; if (!j.enabled) continue;
            const auto f = jointFeedback(j);
            bool fresh = f.valid && f.state == 2 && !f.fault && std::chrono::duration<double>(now - f.received).count() <= feedback_timeout_;
            const double ep = 2 * j.limits.position / 65535, ev = 2 * j.limits.velocity / 65535;
            const double position_error = std::abs(j.command.position - f.position), velocity_error = std::abs(j.command.velocity - f.velocity);
            const double pp = task_active_ ? task_path_position_[i] : j.path_position;
            const double pv = task_active_ ? task_path_velocity_[i] : j.path_velocity;
            if (fresh && (position_error > pp + ep || velocity_error > pv + ev)) {
                if (j.error_since == lingzu::Clock::time_point{}) j.error_since = now;
                if (std::chrono::duration<double>(now - j.error_since).count() >= tracking_timeout_)
                    failure = "Persistent tracking error at " + j.name + ": position=" + std::to_string(position_error) + ", velocity=" + std::to_string(velocity_error);
            } else j.error_since = {};
            if (arrived && std::find(task_joints_.begin(), task_joints_.end(), i) != task_joints_.end() &&
                (!fresh || f.received < settling_since_ || position_error > task_goal_position_[i] + ep || velocity_error > task_goal_velocity_[i] + ev)) arrived = false;
        }
    }
    if (!failure.empty()) { disable(failure, Action::Result::PATH_TOLERANCE_VIOLATED); return; }
    if (!task_active_ || motion_->active()) return;
    if (arrived) {
        if (within_goal_since_ == lingzu::Clock::time_point{}) within_goal_since_ = now;
        if (std::chrono::duration<double>(now - within_goal_since_).count() >= settle_time_) {
            bool canceling = cancel_requested_;
            finishTask(canceling ? "canceled" : "completed", canceling ? "Measured stop confirmed" : "Measured joint arrival confirmed", Action::Result::SUCCESSFUL);
            motion_status_ = canceling ? "idle" : "completed";
            return;
        }
    } else within_goal_since_ = {};
    if (std::chrono::duration<double>(now - settling_since_).count() >= task_goal_wait_)
        disable("Final joint position/velocity did not settle before the goal deadline", Action::Result::GOAL_TOLERANCE_VIOLATED);
}
void Tangair_usb2can::publishState() {
    sensor_msgs::msg::JointState actual, desired;
    actual.header.stamp = node_->now(); desired.header.stamp = actual.header.stamp;
    usb2can_demo_lingzu::msg::TrajectoryTask task; task.header = actual.header;
    task.task_id = task_id_; task.state = task_state_; task.message = task_message_; task.active = task_active_;
    task.planned_duration = task_duration_; task.elapsed_time = task_active_ ? motion_->elapsed() : task_elapsed_;
    task.progress = task_duration_ > 0 ? std::min(1.0, task.elapsed_time / task_duration_) : (task_id_.empty() ? 0.0 : 1.0);
    if (task_state_ == "completed") task.progress = 1;
    const auto now = lingzu::Clock::now(); const double unavailable = std::numeric_limits<double>::quiet_NaN();
    auto feedback = std::make_shared<Action::Feedback>(); feedback->header = actual.header;
    { std::lock_guard<std::mutex> lock(feedback_mutex_);
      for (std::size_t i = 0; i < joints_.size(); ++i) { const auto& j = joints_[i]; auto f = jointFeedback(j);
          const bool valid = f.valid && std::chrono::duration<double>(now - f.received).count() <= feedback_timeout_;
          actual.name.push_back(j.name); desired.name.push_back(j.name);
          actual.position.push_back(valid ? f.position : unavailable); actual.velocity.push_back(valid ? f.velocity : unavailable); actual.effort.push_back(valid ? f.torque : unavailable);
          desired.position.push_back(j.command.position); desired.velocity.push_back(j.enabled ? j.command.velocity : 0);
      }
      for (auto i : task_joints_) { const auto& j = joints_[i]; auto f = jointFeedback(j);
          const bool valid = f.valid && std::chrono::duration<double>(now - f.received).count() <= feedback_timeout_;
          const double pos = valid ? f.position : unavailable, vel = valid ? f.velocity : unavailable;
          task.joint_names.push_back(j.name); task.desired_positions.push_back(j.command.position); task.actual_positions.push_back(pos); task.position_errors.push_back(j.command.position - pos);
          feedback->joint_names.push_back(j.name);
          feedback->desired.positions.push_back(j.command.position); feedback->desired.velocities.push_back(j.command.velocity);
          feedback->actual.positions.push_back(pos); feedback->actual.velocities.push_back(vel);
          feedback->error.positions.push_back(j.command.position - pos); feedback->error.velocities.push_back(j.command.velocity - vel);
      }
    }
    const auto elapsed_ns = static_cast<int64_t>(motion_->elapsed() * 1e9);
    feedback->desired.time_from_start.sec = static_cast<int32_t>(elapsed_ns / 1000000000);
    feedback->desired.time_from_start.nanosec = static_cast<uint32_t>(elapsed_ns % 1000000000);
    feedback->actual.time_from_start = feedback->error.time_from_start = feedback->desired.time_from_start;
    if (goal_ && task_active_) goal_->publish_feedback(feedback);
    state_pub_->publish(actual); command_pub_->publish(desired); task_pub_->publish(task);
    std_msgs::msg::String status; status.data = motion_status_; status_pub_->publish(status);
}
