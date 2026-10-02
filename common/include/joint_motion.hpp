// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <ruckig/ruckig.hpp>
#include <string>
#include <vector>

namespace lingzu {
struct JointMotionLimits {
    std::string name;
    double lower, upper, velocity, acceleration, jerk;
};
struct MotionState {
    std::vector<double> position, velocity, acceleration;
    explicit MotionState(std::size_t size) : position(size), velocity(size), acceleration(size) {}
};
struct MotionPoint {
    double time;
    std::vector<double> positions, velocities, accelerations;
};

// All state is owned by the controller's single-threaded executor.
class JointMotion {
    using Curve = ruckig::Trajectory<ruckig::DynamicDOFs>;
    using Input = ruckig::InputParameter<ruckig::DynamicDOFs>;
    struct Segment { double start; Curve curve; };
    std::vector<JointMotionLimits> limits_;
    MotionState state_;
    ruckig::Ruckig<ruckig::DynamicDOFs> generator_;
    std::vector<Segment> segments_;
    double elapsed_ = 0, duration_ = 0;
    std::size_t segment_ = 0;
    Input input(const MotionState& start) const;
    Curve calculate(const Input& input);
    void validateState(const MotionState& state) const;
    void commit(std::vector<Segment> segments, double duration);
public:
    explicit JointMotion(std::vector<JointMotionLimits> limits);
    const MotionState& state() const { return state_; }
    double duration() const { return duration_; }
    double elapsed() const { return elapsed_; }
    bool active() const { return !segments_.empty(); }
    void reset(const MotionState& state);
    void clear();
    double plan(const std::vector<std::string>& names, const std::vector<MotionPoint>& points, bool retime);
    void move(const std::vector<double>& positions);
    void brake();
    bool advance(double seconds);
};
} // namespace lingzu
