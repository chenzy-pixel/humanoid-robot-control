// Copyright (c) 2023-2025 TANGAIR
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "lingzu_protocol.hpp"
#include "usb_can.h"
#include <atomic>
#include <cstddef>
#include <functional>
#include <mutex>
#include <thread>

static_assert(sizeof(FrameInfo) == 6, "USB2CAN SDK structure ABI mismatch");
static_assert(offsetof(FrameInfo, frameType) == 4, "USB2CAN frameType offset mismatch");
static_assert(offsetof(FrameInfo, dataLength) == 5, "USB2CAN DLC offset mismatch");

namespace lingzu {
class Transport {
    std::array<int32_t, 2> devices_{{-1, -1}};
    SendSchedule schedule_;
    std::mutex write_mutex_;
public:
    std::atomic<bool> failed{false};
    Transport() = default;
    Transport(const Transport&) = delete;
    Transport& operator=(const Transport&) = delete;
    ~Transport() { close(); }
    bool open(const std::array<std::string, 2>& paths) {
        close();
        for (unsigned i = 0; i < 2; ++i) devices_[i] = openUSBCAN(paths[i].c_str());
        return devices_[0] >= 0 || devices_[1] >= 0;
    }
    bool available(unsigned device) const { return devices_.at(device) >= 0; }
    void close() {
        for (auto& device : devices_) {
            if (device >= 0) closeUSBCAN(device);
            device = -1;
        }
    }
    bool send(unsigned device, uint8_t channel, const Frame& frame, int attempts = 1) {
        if (device >= 2 || channel < 1 || channel > 2 || frame.length > 8 ||
            !available(device) || attempts < 1) return false;
        std::lock_guard<std::mutex> guard(write_mutex_);
        FrameInfo info{};
        info.canID = frame.id;
        info.frameType = frame.extended ? EXTENDED : STANDARD;
        info.dataLength = frame.length;
        auto data = frame.data;
        const unsigned bus = device * 2 + channel - 1;
        for (int attempt = 0; attempt < attempts; ++attempt) {
            std::this_thread::sleep_until(schedule_.due(bus));
            const auto result = sendUSBCAN(devices_[device], channel, &info, data.data());
            schedule_.sent(bus, Clock::now(), 300);
            // The reviewed classic transport returns the complete fixed 17-byte packet.
            if (result == 17) return true;
        }
        failed.store(true);
        return false;
    }
    bool receive(unsigned device, uint8_t& channel, Frame& frame) {
        if (device >= 2 || !available(device)) return false;
        FrameInfo info{};
        Frame f;
        if (readUSBCAN(devices_[device], &channel, &info, f.data.data(), 100000) != 0) return false;
        if (info.frameType > 1 || info.dataLength > 8 || channel < 1 || channel > 2) return false;
        f.id = info.canID;
        f.extended = info.frameType == EXTENDED;
        f.length = info.dataLength;
        frame = f;
        return true;
    }
};
} // namespace lingzu
