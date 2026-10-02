// Exercise the actual Ruckig calculator and the complete local motion planner.
#include "joint_motion.hpp"
#include <cassert>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>

namespace {
int checks = 0;
void rejected(const std::function<void()>& operation) {
    bool failed = false;
    try { operation(); } catch (const std::exception&) { failed = true; }
    assert(failed); ++checks;
}
std::vector<lingzu::JointMotionLimits> limits() {
    return {{"a", -1, 1, 0.5, 1.0, 4.0}, {"b", -1, 1, 0.3, 0.6, 2.0}, {"c", -1, 1, 0.4, 0.8, 3.0}};
}
void bounded(lingzu::JointMotion& motion, double dt = 0.001) {
    int samples = 0;
    while (motion.active()) {
        const auto before = motion.state();
        motion.advance(dt);
        const auto& after = motion.state();
        for (std::size_t i = 0; i < limits().size(); ++i) {
            const auto l = limits()[i];
            assert(after.position[i] >= l.lower - 1e-8 && after.position[i] <= l.upper + 1e-8);
            assert(std::abs(after.velocity[i]) <= l.velocity + 1e-8);
            assert(std::abs(after.acceleration[i]) <= l.acceleration + 1e-8);
            assert(std::abs(after.position[i] - before.position[i]) <= l.velocity * dt + 1e-8);
            assert(std::abs(after.velocity[i] - before.velocity[i]) <= l.acceleration * dt + 1e-8);
            assert(std::abs(after.acceleration[i] - before.acceleration[i]) <= l.jerk * dt + 1e-7);
        }
        assert(++samples < 100000);
    }
    ++checks;
}
}
int main() {
    using lingzu::MotionPoint;
    lingzu::JointMotion motion(limits());
    lingzu::MotionState start(3); start.position = {0.1, -0.2, 0.3}; motion.reset(start);
    rejected([] { lingzu::JointMotion invalid({}); });
    auto bad_limits = limits(); bad_limits[1].name = "a";
    rejected([&] { lingzu::JointMotion invalid(bad_limits); });
    bad_limits = limits(); bad_limits[1].acceleration = 0;
    rejected([&] { lingzu::JointMotion invalid(bad_limits); });
    rejected([&] { motion.plan({"unknown"}, {{2, {0}, {}, {}}}, false); });
    rejected([&] { motion.plan({"a", "a"}, {{2, {0, 0}, {}, {}}}, false); });
    rejected([&] { motion.plan({"a"}, {{2, {}, {}, {}}}, false); });
    rejected([&] { motion.plan({"a"}, {{2, {0}, {0, 0}, {}}}, false); });
    rejected([&] { motion.plan({"a"}, {{-1, {0}, {}, {}}}, false); });
    rejected([&] { motion.plan({"a"}, {{2, {0}, {}, {}}, {1, {0}, {}, {}}}, false); });
    rejected([&] { motion.plan({"a"}, {{2, {std::numeric_limits<double>::quiet_NaN()}, {}, {}}}, false); });
    rejected([&] { motion.plan({"a"}, {{2, {2}, {}, {}}}, false); });
    rejected([&] { motion.plan({"a"}, {{0, {0.2}, {}, {}}}, false); });
    rejected([&] { motion.plan({"a"}, {{2, {0.2}, {0.1}, {}}}, false); });
    rejected([&] { motion.plan({"a"}, {{0.01, {0.5}, {}, {}}}, false); });
    const double retimed = motion.plan({"a"}, {{0.01, {0.5}, {}, {}}}, true);
    assert(retimed > 0.01); ++checks;
    bounded(motion);
    assert(std::abs(motion.state().position[0] - 0.5) < 1e-8); ++checks;

    motion.reset(start);
    assert(std::abs(motion.plan({"a"}, {{2, {0.1}, {}, {}}}, false) - 2) < 1e-6); ++checks;
    bounded(motion); motion.reset(start);
    assert(std::abs(motion.plan({"b", "a"}, {{3, {0.2, -0.3}, {}, {}}, {6, {-0.1, 0.2}, {}, {}}}, false) - 6) < 1e-6); ++checks;
    motion.advance(3);
    assert(std::abs(motion.state().position[0] + 0.3) < 1e-8 && std::abs(motion.state().position[1] - 0.2) < 1e-8);
    assert(std::abs(motion.state().velocity[0]) < 1e-8 && std::abs(motion.state().acceleration[1]) < 1e-8); ++checks;
    bounded(motion);
    assert(std::abs(motion.state().position[0] - 0.2) < 1e-8 && std::abs(motion.state().position[1] + 0.1) < 1e-8);
    assert(std::abs(motion.state().position[2] - 0.3) < 1e-8); ++checks;

    motion.reset(start);
    motion.plan({"a"}, {{0, {0.1}, {}, {}}, {3, {0.4}, {}, {}}}, false); ++checks;
    motion.advance(0.8);
    const auto moving = motion.state();
    const double old_duration = motion.duration();
    rejected([&] { motion.plan({"a"}, {{1, {0.3}, {}, {}}, {2, {2}, {}, {}}}, true); });
    assert(motion.active() && motion.duration() == old_duration && motion.state().position == moving.position); ++checks;
    motion.plan({"a"}, {{3, {-0.4}, {}, {}}}, false);
    assert(motion.state().position == moving.position && motion.state().velocity == moving.velocity && motion.state().acceleration == moving.acceleration); ++checks;
    bounded(motion);
    assert(std::abs(motion.state().position[0] + 0.4) < 1e-8); ++checks;

    motion.reset(start);
    motion.plan({"a", "b"}, {{2, {0.3, 0.05}, {0.08, 0.05}, {0, 0}}, {4, {0.6, 0.25}, {}, {}}}, false);
    motion.advance(2);
    assert(std::abs(motion.state().velocity[0] - 0.08) < 1e-8 && std::abs(motion.state().velocity[1] - 0.05) < 1e-8); ++checks;
    bounded(motion);

    motion.reset(start); motion.move({0.8, 0.5, 0.3}); motion.advance(0.5);
    const auto before_brake = motion.state(); motion.brake();
    assert(motion.state().velocity == before_brake.velocity && motion.state().acceleration == before_brake.acceleration); ++checks;
    bounded(motion);
    assert(motion.state().position[0] < 0.8 && motion.state().position[1] < 0.5); ++checks;
    const auto held = motion.state(); motion.advance(10);
    assert(motion.state().position == held.position && !motion.active()); ++checks;

    // Both waypoint endpoints fit, but a high intermediate velocity would overshoot the position limit.
    motion.reset(start);
    rejected([&] { motion.plan({"a"}, {{3, {0.999}, {0.5}, {0}}, {6, {0.1}, {}, {}}}, true); });
    auto near_edge = start; near_edge.position[0] = 0.999; near_edge.velocity[0] = 0.4; motion.reset(near_edge);
    rejected([&] { motion.brake(); });
    motion.reset(start); motion.move({0.8, 0.2, 0.3}); motion.advance(0.3);
    motion.move({-0.5, -0.4, 0.3}); bounded(motion); ++checks;
    motion.clear(); assert(!motion.active()); ++checks;
    rejected([&] { motion.advance(-1); });
    rejected([&] { motion.advance(std::numeric_limits<double>::infinity()); });
    std::cout << checks << " joint trajectory checks passed, including dense position/velocity/acceleration/jerk sampling\n";
}
