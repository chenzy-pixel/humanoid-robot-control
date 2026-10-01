// Copyright (c) 2023-2025 TANGAIR
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace lingzu {
using Clock = std::chrono::steady_clock;
struct Limits { double position, velocity, torque, kp, kd; };
inline Limits limits(const std::string& model) {
    if (model == "RS00") return {12.57, 33, 14, 500, 5};
    if (model == "RS02") return {4 * std::acos(-1.0), 44, 17, 500, 5};
    if (model == "RS03") return {12.57, 20, 60, 5000, 100};
    if (model == "RS04") return {12.57, 15, 120, 5000, 100};
    throw std::invalid_argument("Select motor model RS00/RS02/RS03/RS04");
}
inline uint16_t encode(double value, double low, double high) {
    if (!std::isfinite(value) || !std::isfinite(low) || !std::isfinite(high) ||
        high <= low || value < low || value > high)
        throw std::invalid_argument("Motor value outside finite protocol range");
    return static_cast<uint16_t>((value - low) * 65535.0 / (high - low));
}
inline double decode(uint16_t value, double low, double high) {
    return value * (high - low) / 65535.0 + low;
}
struct Frame {
    uint32_t id = 0;
    bool extended = true;
    uint8_t length = 8;
    std::array<uint8_t, 8> data{};
};
struct Command { double position = 0, velocity = 0, kp = 0, kd = 0, torque = 0; };
inline void put16(Frame& f, int offset, uint16_t value) {
    f.data[offset] = static_cast<uint8_t>(value >> 8);
    f.data[offset + 1] = static_cast<uint8_t>(value);
}
inline uint16_t get16(const Frame& f, int offset) {
    return static_cast<uint16_t>((f.data[offset] << 8) | f.data[offset + 1]);
}
inline Frame control(uint8_t motor, const Limits& p, const Command& c) {
    if (!motor || motor == 255) throw std::invalid_argument("Motor ID must be 1..254");
    Frame f;
    f.id = (1u << 24) | (static_cast<uint32_t>(encode(c.torque, -p.torque, p.torque)) << 8) | motor;
    put16(f, 0, encode(c.position, -p.position, p.position));
    put16(f, 2, encode(c.velocity, -p.velocity, p.velocity));
    put16(f, 4, encode(c.kp, 0, p.kp));
    put16(f, 6, encode(c.kd, 0, p.kd));
    return f;
}
inline Frame special(uint8_t type, uint8_t motor, uint8_t host, bool flag = false) {
    if (type != 0 && type != 3 && type != 4 && type != 6)
        throw std::invalid_argument("Unsupported command type");
    Frame f;
    f.id = (static_cast<uint32_t>(type) << 24) | (static_cast<uint32_t>(host) << 8) | motor;
    f.data[0] = flag ? 1 : 0;
    return f;
}
inline Frame run_mode(uint8_t motor, uint8_t host, uint8_t mode) {
    if (mode != 0 && mode != 1 && mode != 2 && mode != 3 && mode != 5)
        throw std::invalid_argument("Invalid motor run mode");
    Frame f;
    f.id = (0x12u << 24) | (static_cast<uint32_t>(host) << 8) | motor;
    f.data[0] = 0x05; f.data[1] = 0x70; f.data[4] = mode;
    return f;
}
struct Feedback {
    bool valid = false;
    uint8_t motor = 0, host = 0, fault = 0, state = 0;
    double position = 0, velocity = 0, torque = 0, temperature = 0;
    Clock::time_point received{};
};
inline bool feedback(const Frame& f, const Limits& p, uint8_t host, Feedback& out) {
    if (!f.extended || f.length != 8 || f.id > 0x1fffffff ||
        ((f.id >> 24) & 31) != 2 || (f.id & 255) != host) return false;
    Feedback decoded;
    decoded.motor = static_cast<uint8_t>(f.id >> 8);
    if (!decoded.motor || decoded.motor == 255) return false;
    decoded.host = host;
    decoded.fault = static_cast<uint8_t>((f.id >> 16) & 63);
    decoded.state = static_cast<uint8_t>((f.id >> 22) & 3);
    decoded.position = decode(get16(f, 0), -p.position, p.position);
    decoded.velocity = decode(get16(f, 2), -p.velocity, p.velocity);
    decoded.torque = decode(get16(f, 4), -p.torque, p.torque);
    decoded.temperature = get16(f, 6) / 10.0;
    decoded.received = Clock::now();
    decoded.valid = true;
    out = decoded;
    return true;
}
// Used by every SDK write, including button commands, retries and shutdown.
class SendSchedule {
    std::array<Clock::time_point, 4> next_{};
public:
    Clock::time_point due(unsigned bus) const { return next_.at(bus); }
    void sent(unsigned bus, Clock::time_point completed, int interval_us) {
        next_.at(bus) = completed + std::chrono::microseconds(interval_us < 300 ? 300 : interval_us);
    }
};
class InputWatchdog {
    Clock::time_point last_{};
    bool seen_ = false;
public:
    void update(Clock::time_point now) { last_ = now; seen_ = true; }
    bool fresh(Clock::time_point now, double timeout) const {
        return seen_ && std::chrono::duration<double>(now - last_).count() <= timeout;
    }
};
} // namespace lingzu
