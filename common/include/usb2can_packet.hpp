// Classic SOULDE USB2CAN wire format. The supplied SDK uses eight payload slots.
#pragma once
#include "lingzu_protocol.hpp"
#include <algorithm>
#include <array>
#include <deque>
#include <vector>

namespace lingzu {
inline uint8_t usb_crc8(const uint8_t* data, std::size_t size) {
    uint8_t crc = 0xFF;
    for (std::size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = static_cast<uint8_t>((crc >> 1) ^ ((crc & 1) ? 0x8C : 0));
    }
    return crc;
}
inline bool usb_valid(uint8_t channel, const Frame& frame) {
    return channel >= 1 && channel <= 2 && frame.length <= 8 &&
        frame.id <= (frame.extended ? 0x1FFFFFFFu : 0x7FFu);
}
inline std::array<uint8_t, 17> usb_packet(uint8_t channel, const Frame& frame) {
    if (!usb_valid(channel, frame)) throw std::invalid_argument("Invalid classic USB2CAN frame");
    std::array<uint8_t, 17> packet{};
    packet[0] = 0xA8; packet[1] = channel;
    for (unsigned i = 0; i < 4; ++i) packet[2 + i] = static_cast<uint8_t>(frame.id >> (8 * i));
    packet[6] = frame.extended ? 1 : 0; packet[7] = frame.length;
    std::copy_n(frame.data.begin(), frame.length, packet.begin() + 8);
    packet[16] = usb_crc8(packet.data(), 16);
    return packet;
}
struct UsbReceived { uint8_t channel; Frame frame; };
class UsbPacketParser {
    std::vector<uint8_t> buffer_;
    std::deque<UsbReceived> frames_;
public:
    void feed(const uint8_t* data, std::size_t size) {
        buffer_.insert(buffer_.end(), data, data + size);
        while (!buffer_.empty()) {
            auto start = std::find(buffer_.begin(), buffer_.end(), uint8_t{0xA9});
            buffer_.erase(buffer_.begin(), start);
            if (buffer_.size() < 17) return;
            Frame frame;
            frame.id = 0;
            for (unsigned i = 0; i < 4; ++i) frame.id |= uint32_t(buffer_[2 + i]) << (8 * i);
            frame.extended = buffer_[6] == 1; frame.length = buffer_[7];
            if (buffer_[6] > 1 || !usb_valid(buffer_[1], frame) ||
                usb_crc8(buffer_.data(), 16) != buffer_[16]) {
                buffer_.erase(buffer_.begin());
                continue;
            }
            std::copy_n(buffer_.begin() + 8, frame.length, frame.data.begin());
            // Bound queued data when a producer supplies many frames in one call.
            if (frames_.size() < 1024) frames_.push_back(UsbReceived{buffer_[1], frame});
            buffer_.erase(buffer_.begin(), buffer_.begin() + 17);
        }
    }
    bool pop(uint8_t& channel, Frame& frame) {
        if (frames_.empty()) return false;
        channel = frames_.front().channel; frame = frames_.front().frame;
        frames_.pop_front(); return true;
    }
};
} // namespace lingzu
