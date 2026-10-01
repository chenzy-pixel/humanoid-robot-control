#include "usb2can_transport.hpp"
#include "usb2can_packet.hpp"
#include <algorithm>
#include <cassert>
#include <iostream>
#include <limits>
#include <vector>

namespace {
struct Write { int device; uint8_t channel; FrameInfo info; std::array<uint8_t, 8> data; lingzu::Clock::time_point at; };
std::vector<Write> writes;
int failures = 0, closes = 0, send_result = 17;
}
extern "C" int32_t openUSBCAN(const char* name) { return std::string(name).find('0') != std::string::npos ? 100 : 101; }
extern "C" int32_t closeUSBCAN(int32_t) { ++closes; return 0; }
extern "C" int32_t sendUSBCAN(int32_t dev, uint8_t channel, FrameInfo* info, uint8_t* data) {
    Write value{}; value.device = dev; value.channel = channel; value.info = *info;
    std::copy(data, data + info->dataLength, value.data.begin()); value.at = lingzu::Clock::now(); writes.push_back(value);
    if (failures) { --failures; return -1; }
    return send_result;
}
extern "C" int32_t readUSBCAN(int32_t, uint8_t*, FrameInfo*, uint8_t*, int32_t) { return -1; }

int main() {
    int checks = 0;
    for (const auto& model : {"RS00", "RS02", "RS03", "RS04"}) {
        auto p = lingzu::limits(model);
        lingzu::Command c;
        c.kp = 18; c.kd = 0.8;
        auto frame = lingzu::control(1, p, c);
        assert(frame.extended && frame.length == 8 && ((frame.id >> 24) & 31) == 1);
        assert(((frame.id >> 8) & 65535) == 32767); ++checks;
        assert(std::abs(lingzu::decode(lingzu::get16(frame, 4), 0, p.kp) - 18) <= p.kp / 65535); ++checks;
        assert(std::abs(lingzu::decode(lingzu::get16(frame, 6), 0, p.kd) - .8) <= p.kd / 65535); ++checks;
        c.position = -p.position; c.torque = -p.torque;
        frame = lingzu::control(1, p, c);
        assert(lingzu::get16(frame, 0) == 0 && ((frame.id >> 8) & 65535) == 0); ++checks;
        c.position = p.position; c.torque = p.torque;
        frame = lingzu::control(1, p, c);
        assert(lingzu::get16(frame, 0) == 65535 && ((frame.id >> 8) & 65535) == 65535); ++checks;
    }
    bool rejected = false;
    try { lingzu::encode(std::numeric_limits<double>::quiet_NaN(), -1, 1); } catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected); ++checks;
    auto mode = lingzu::run_mode(1, 0xFD, 0);
    assert(mode.id == 0x1200FD01 && mode.data[0] == 5 && mode.data[1] == 0x70 && mode.data[4] == 0); ++checks;
    lingzu::Frame frame; frame.id = (2u << 24) | (2u << 22) | (1u << 8) | 0xFD;
    for (int i : {0, 2, 4}) lingzu::put16(frame, i, 32767);
    lingzu::put16(frame, 6, 300);
    lingzu::Feedback state;
    assert(lingzu::feedback(frame, lingzu::limits("RS00"), 0xFD, state) && state.fault == 0 && state.state == 2 && state.temperature == 30); ++checks;
    assert(!lingzu::feedback(frame, lingzu::limits("RS00"), 0, state)); ++checks;
    frame.length = 7; assert(!lingzu::feedback(frame, lingzu::limits("RS00"), 0xFD, state)); ++checks;
    frame.length = 8; frame.id = (0x15u << 24) | (1u << 8) | 0xFD;
    assert(!lingzu::feedback(frame, lingzu::limits("RS00"), 0xFD, state)); ++checks;
    frame.id = (2u << 24) | (1u << 8) | 0xFD; frame.extended = false;
    assert(!lingzu::feedback(frame, lingzu::limits("RS00"), 0xFD, state)); ++checks;

    lingzu::SendSchedule schedule; auto origin = lingzu::Clock::now();
    schedule.sent(0, origin, 75);
    assert(schedule.due(0) == origin + std::chrono::microseconds(300)); ++checks;
    assert(schedule.due(1) != schedule.due(0)); ++checks;
    lingzu::InputWatchdog watchdog;
    assert(!watchdog.fresh(origin, .5)); watchdog.update(origin);
    assert(watchdog.fresh(origin + std::chrono::milliseconds(499), .5));
    assert(!watchdog.fresh(origin + std::chrono::milliseconds(501), .5)); checks += 3;

    const std::array<uint8_t, 12> manual{{0xA8, 1, 0xFF, 7, 0, 0, 0, 4, 0xFF, 0xFF, 0, 0x82}};
    assert(lingzu::usb_crc8(manual.data(), manual.size()) == 0xE1); ++checks;
    lingzu::Frame short_frame; short_frame.id = 0x7FF; short_frame.extended = false; short_frame.length = 4;
    std::copy(manual.begin() + 8, manual.end(), short_frame.data.begin());
    auto packet = lingzu::usb_packet(1, short_frame);
    assert(packet[16] == 0xC6 && packet[12] == 0 && packet[15] == 0); ++checks;
    packet[0] = 0xA9; packet[16] = lingzu::usb_crc8(packet.data(), 16);
    for (std::size_t split = 0; split <= packet.size(); ++split) {
        lingzu::UsbPacketParser parser; uint8_t channel = 0; lingzu::Frame read;
        parser.feed(packet.data(), split); parser.feed(packet.data() + split, packet.size() - split);
        assert(parser.pop(channel, read) && channel == 1 && read.id == 0x7FF && read.length == 4 && read.data[3] == 0x82);
        assert(!parser.pop(channel, read)); ++checks;
    }
    for (unsigned field : {0u, 1u, 4u, 6u, 7u, 16u}) {
        auto broken = packet; broken[field] ^= 0x80;
        if (field != 16) broken[16] = lingzu::usb_crc8(broken.data(), 16);
        lingzu::UsbPacketParser parser; uint8_t channel; lingzu::Frame read;
        parser.feed(broken.data(), 17); parser.feed(packet.data(), 17); parser.feed(packet.data(), 17);
        assert(parser.pop(channel, read) && read.id == 0x7FF);
        assert(parser.pop(channel, read) && !parser.pop(channel, read)); ++checks;
    }

    {
        lingzu::Transport bus; assert(bus.open({{"device0", "device1"}}));
        auto stop = lingzu::special(4, 1, 0);
        failures = 2; assert(bus.send(0, 1, stop, 3)); ++checks;
        assert(writes.size() == 3); ++checks;
        assert(bus.send(0, 2, stop)); assert(bus.send(0, 1, stop));
        for (std::size_t i = 1; i < writes.size(); ++i) {
            if (writes[i].channel == writes[i-1].channel)
                assert(writes[i].at - writes[i-1].at >= std::chrono::microseconds(300));
        }
        assert(writes.back().at - writes[2].at >= std::chrono::microseconds(300)); ++checks;
        failures = 3; assert(!bus.send(1, 1, stop, 3) && bus.failed.load()); ++checks;
        for (int partial : {0, 8, 16}) {
            send_result = partial; assert(!bus.send(0, 1, stop)); ++checks;
        }
        send_result = 17; assert(bus.send(0, 1, stop)); ++checks;
        uint8_t channel = 0; assert(!bus.receive(0, channel, frame)); ++checks;
        assert(!bus.receive(2, channel, frame)); ++checks;
    }
    assert(closes == 2); ++checks;
    std::cout << checks << " C++ protocol/transport checks passed\n";
}
