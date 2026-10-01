// Linux SDK-compatible classic USB2CAN transport; no motor commands on open.
#include "usb_can.h"
#include "usb2can_packet.hpp"
#include <cerrno>
#include <chrono>
#include <fcntl.h>
#include <map>
#include <memory>
#include <mutex>
#include <poll.h>
#include <sys/file.h>
#include <termios.h>
#include <unistd.h>

namespace {
struct Device {
    int fd;
    std::mutex read_mutex, write_mutex;
    lingzu::UsbPacketParser parser;
    explicit Device(int value) : fd(value) {}
};
std::mutex devices_mutex;
std::map<int, std::shared_ptr<Device>> devices;
std::shared_ptr<Device> get_device(int fd) {
    std::lock_guard<std::mutex> lock(devices_mutex);
    auto found = devices.find(fd);
    if (found != devices.end()) return found->second;
    errno = EBADF; return {};
}
using Clock = std::chrono::steady_clock;
bool wait_for(int fd, short event, Clock::time_point deadline) {
    for (;;) {
        auto left = std::chrono::duration_cast<std::chrono::microseconds>(deadline - Clock::now()).count();
        if (left <= 0) { errno = ETIMEDOUT; return false; }
        struct pollfd request{fd, event, 0};
        int timeout = static_cast<int>((left + 999) / 1000);
        int ready = ::poll(&request, 1, timeout);
        if (ready < 0 && errno == EINTR) continue;
        if (ready <= 0) { if (ready == 0) errno = ETIMEDOUT; return false; }
        if (request.revents & (POLLERR | POLLHUP | POLLNVAL)) { errno = EIO; return false; }
        if (request.revents & event) return true;
    }
}
}

extern "C" int32_t openUSBCAN(const char* name) {
    if (!name || !*name) { errno = EINVAL; return -1; }
    int fd = ::open(name, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) return -1;
    auto fail = [fd]() { int error = errno; ::close(fd); errno = error; return int32_t{-1}; };
    if (::flock(fd, LOCK_EX | LOCK_NB) != 0) return fail();
    struct termios config{};
    if (::tcgetattr(fd, &config) != 0) return fail();
    ::cfmakeraw(&config);
    config.c_cflag |= CLOCAL | CREAD;
    config.c_cflag &= ~CRTSCTS;
    config.c_cc[VMIN] = 0; config.c_cc[VTIME] = 0;
    // USB CDC ignores the nominal serial speed; CAN speed is fixed at 1 Mbit/s.
    if (::cfsetispeed(&config, B115200) != 0 || ::cfsetospeed(&config, B115200) != 0 ||
        ::tcsetattr(fd, TCSANOW, &config) != 0) return fail();
    std::lock_guard<std::mutex> lock(devices_mutex);
    devices.emplace(fd, std::make_shared<Device>(fd));
    return fd;
}
extern "C" int32_t closeUSBCAN(int32_t fd) {
    auto device = get_device(fd);
    if (!device) return -1;
    std::unique_lock<std::mutex> read(device->read_mutex, std::defer_lock);
    std::unique_lock<std::mutex> write(device->write_mutex, std::defer_lock);
    std::lock(read, write);
    if (device->fd < 0) { errno = EBADF; return -1; }
    std::lock_guard<std::mutex> lock(devices_mutex);
    devices.erase(fd);
    int result = ::close(device->fd); device->fd = -1;
    return result;
}
extern "C" int32_t sendUSBCAN(int32_t fd, uint8_t channel, FrameInfo* info, uint8_t* data) {
    if (!info || (!data && info->dataLength) || info->frameType > 1) { errno = EINVAL; return -1; }
    lingzu::Frame frame; frame.id = info->canID; frame.extended = info->frameType == EXTENDED;
    frame.length = info->dataLength;
    if (!lingzu::usb_valid(channel, frame)) { errno = EINVAL; return -1; }
    if (frame.length) std::copy_n(data, frame.length, frame.data.begin());
    auto packet = lingzu::usb_packet(channel, frame);
    auto device = get_device(fd);
    if (!device) return -1;
    std::lock_guard<std::mutex> lock(device->write_mutex);
    if (device->fd < 0) { errno = EBADF; return -1; }
    auto deadline = Clock::now() + std::chrono::seconds(1);
    std::size_t sent = 0;
    while (sent < packet.size()) {
        if (Clock::now() >= deadline) { errno = ETIMEDOUT; return -1; }
        auto count = ::write(fd, packet.data() + sent, packet.size() - sent);
        if (count > 0) { sent += static_cast<std::size_t>(count); continue; }
        if (count < 0 && errno == EINTR) continue;
        if (count == 0) { errno = EIO; return -1; }
        if (errno != EAGAIN && errno != EWOULDBLOCK) return -1;
        if (!wait_for(fd, POLLOUT, deadline)) return -1;
    }
    return static_cast<int32_t>(sent);
}
extern "C" int32_t readUSBCAN(int32_t fd, uint8_t* channel, FrameInfo* info, uint8_t* data, int32_t timeout) {
    if (!channel || !info || !data || timeout < 0) { errno = EINVAL; return -1; }
    auto device = get_device(fd);
    if (!device) return -1;
    std::lock_guard<std::mutex> lock(device->read_mutex);
    if (device->fd < 0) { errno = EBADF; return -1; }
    auto deadline = Clock::now() + std::chrono::microseconds(timeout);
    lingzu::Frame frame;
    while (!device->parser.pop(*channel, frame)) {
        uint8_t bytes[4096];
        auto count = ::read(fd, bytes, sizeof(bytes));
        if (count > 0) { device->parser.feed(bytes, static_cast<std::size_t>(count)); }
        else {
            if (count < 0 && errno == EINTR) {
                if (Clock::now() >= deadline) { errno = ETIMEDOUT; return -1; }
                continue;
            }
            if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK) return -1;
            if (!wait_for(fd, POLLIN, deadline)) return -1;
        }
        if (Clock::now() >= deadline) {
            if (!device->parser.pop(*channel, frame)) { errno = ETIMEDOUT; return -1; }
            break;
        }
    }
    info->canID = frame.id; info->frameType = frame.extended ? EXTENDED : STANDARD;
    info->dataLength = frame.length;
    std::copy(frame.data.begin(), frame.data.end(), data);
    return 0;
}
