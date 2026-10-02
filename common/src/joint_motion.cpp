// SPDX-License-Identifier: Apache-2.0
#include "joint_motion.hpp"
#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <utility>

namespace lingzu {
namespace {
constexpr double tolerance = 1e-8;
bool positive(double value) { return std::isfinite(value) && value > 0; }
}
JointMotion::JointMotion(std::vector<JointMotionLimits> limits)
    : limits_(std::move(limits)), state_(limits_.size()), generator_(limits_.size()) {
    if (limits_.empty()) throw std::invalid_argument("Empty motion configuration");
    std::set<std::string> names;
    for (const auto& limit : limits_) {
        if (limit.name.empty() || !names.insert(limit.name).second ||
            !std::isfinite(limit.lower) || !std::isfinite(limit.upper) || limit.lower >= limit.upper ||
            !positive(limit.velocity) || !positive(limit.acceleration) || !positive(limit.jerk))
            throw std::invalid_argument("Invalid joint motion limits");
    }
}
void JointMotion::validateState(const MotionState& state) const {
    if (state.position.size() != limits_.size() || state.velocity.size() != limits_.size() ||
        state.acceleration.size() != limits_.size()) throw std::invalid_argument("Motion state size mismatch");
    for (std::size_t i = 0; i < limits_.size(); ++i) {
        const auto& limit = limits_[i];
        if (!std::isfinite(state.position[i]) || !std::isfinite(state.velocity[i]) || !std::isfinite(state.acceleration[i]) ||
            state.position[i] < limit.lower - tolerance || state.position[i] > limit.upper + tolerance ||
            std::abs(state.velocity[i]) > limit.velocity + tolerance ||
            std::abs(state.acceleration[i]) > limit.acceleration + tolerance)
            throw std::invalid_argument("Joint state outside motion limits: " + limit.name);
    }
}
JointMotion::Input JointMotion::input(const MotionState& start) const {
    Input in(limits_.size());
    in.current_position = start.position; in.current_velocity = start.velocity; in.current_acceleration = start.acceleration;
    for (std::size_t i = 0; i < limits_.size(); ++i) {
        in.max_velocity[i] = limits_[i].velocity;
        in.max_acceleration[i] = limits_[i].acceleration;
        in.max_jerk[i] = limits_[i].jerk;
    }
    in.synchronization = ruckig::Synchronization::Time;
    return in;
}
JointMotion::Curve JointMotion::calculate(const Input& in) {
    Curve curve(limits_.size());
    if (!generator_.validate_input(in, true, true) || generator_.calculate(in, curve) < ruckig::Result::Working)
        throw std::invalid_argument("Cannot generate a trajectory within the configured motion limits");
    if (!std::isfinite(curve.get_duration()) || curve.get_duration() > 3600)
        throw std::invalid_argument("Trajectory segment exceeds one hour");
    const auto extrema = curve.get_position_extrema();
    for (std::size_t i = 0; i < limits_.size(); ++i)
        if (extrema[i].min < limits_[i].lower - tolerance || extrema[i].max > limits_[i].upper + tolerance)
            throw std::invalid_argument("Interpolated trajectory crosses mechanical limits: " + limits_[i].name);
    return curve;
}
void JointMotion::commit(std::vector<Segment> segments, double duration) {
    segments_ = std::move(segments); duration_ = duration; elapsed_ = 0; segment_ = 0;
}
void JointMotion::clear() { commit({}, 0); }
void JointMotion::reset(const MotionState& state) { validateState(state); state_ = state; clear(); }
double JointMotion::plan(const std::vector<std::string>& names, const std::vector<MotionPoint>& points, bool retime) {
    if (names.empty() || points.empty() || points.size() > 100)
        throw std::invalid_argument("Use named joints and 1..100 trajectory points");
    std::vector<std::size_t> indices;
    std::set<std::string> unique;
    for (const auto& name : names) {
        auto found = std::find_if(limits_.begin(), limits_.end(), [&](const auto& limit) { return limit.name == name; });
        if (found == limits_.end() || !unique.insert(name).second)
            throw std::invalid_argument("Unknown or duplicate trajectory joint: " + name);
        indices.push_back(static_cast<std::size_t>(found - limits_.begin()));
    }
    MotionState start = state_;
    std::vector<Segment> planned;
    double previous = 0, total = 0;
    for (std::size_t p = 0; p < points.size(); ++p) {
        const auto& point = points[p];
        if (!std::isfinite(point.time) || point.time < 0 || point.time > 3600 ||
            (p > 0 && point.time <= previous) || point.positions.size() != names.size() ||
            (!point.velocities.empty() && point.velocities.size() != names.size()) ||
            (!point.accelerations.empty() && point.accelerations.size() != names.size()))
            throw std::invalid_argument("Invalid trajectory point sizes or time_from_start");
        MotionState target(limits_.size());
        target.position = start.position;
        for (std::size_t n = 0; n < indices.size(); ++n) {
            const auto i = indices[n];
            target.position[i] = point.positions[n];
            target.velocity[i] = point.velocities.empty() ? 0 : point.velocities[n];
            target.acceleration[i] = point.accelerations.empty() ? 0 : point.accelerations[n];
        }
        validateState(target);
        if (p + 1 == points.size())
            for (std::size_t i = 0; i < limits_.size(); ++i)
                if (target.velocity[i] != 0 || target.acceleration[i] != 0)
                    throw std::invalid_argument("Final trajectory point must have zero velocity and acceleration");
        if (point.time == 0) {
            for (std::size_t i = 0; i < limits_.size(); ++i)
                if (std::abs(start.position[i] - target.position[i]) > tolerance ||
                    std::abs(start.velocity[i] - target.velocity[i]) > tolerance ||
                    std::abs(start.acceleration[i] - target.acceleration[i]) > tolerance)
                    throw std::invalid_argument("A zero-time point must match the current command state");
        } else {
            Input in = input(start);
            in.target_position = target.position; in.target_velocity = target.velocity; in.target_acceleration = target.acceleration;
            in.minimum_duration = point.time - previous;
            auto curve = calculate(in);
            const double duration = curve.get_duration();
            if (!retime && duration > *in.minimum_duration + 1e-6)
                throw std::invalid_argument("Trajectory timing exceeds motion limits; increase times or enable retime_trajectory");
            planned.push_back({total, std::move(curve)});
            total += duration;
            if (total > 3600) throw std::invalid_argument("Retimed trajectory exceeds one hour");
        }
        start = std::move(target); previous = point.time;
    }
    // Commit only after every segment passes validation, preserving the active plan on rejection.
    commit(std::move(planned), total);
    return total;
}
void JointMotion::move(const std::vector<double>& positions) {
    MotionState target(limits_.size()); target.position = positions; validateState(target);
    Input in = input(state_); in.target_position = positions;
    auto curve = calculate(in);
    const double duration = curve.get_duration();
    commit({Segment{0, std::move(curve)}}, duration);
}
void JointMotion::brake() {
    Input in = input(state_);
    in.control_interface = ruckig::ControlInterface::Velocity;
    auto curve = calculate(in);
    const double duration = curve.get_duration();
    commit({Segment{0, std::move(curve)}}, duration);
}
bool JointMotion::advance(double seconds) {
    if (!std::isfinite(seconds) || seconds < 0) throw std::invalid_argument("Invalid motion time step");
    if (!active()) return false;
    elapsed_ = std::min(duration_, elapsed_ + seconds);
    while (segment_ + 1 < segments_.size() && elapsed_ >= segments_[segment_ + 1].start) ++segment_;
    const auto& segment = segments_[segment_];
    segment.curve.at_time(std::min(segment.curve.get_duration(), elapsed_ - segment.start),
                          state_.position, state_.velocity, state_.acceleration);
    validateState(state_);
    if (elapsed_ >= duration_) {
        std::fill(state_.velocity.begin(), state_.velocity.end(), 0);
        std::fill(state_.acceleration.begin(), state_.acceleration.end(), 0);
        segments_.clear(); return true;
    }
    return false;
}
} // namespace lingzu
