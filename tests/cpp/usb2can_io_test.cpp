#include "usb_can.h"
#include "usb2can_packet.hpp"
#include "test_io.hpp"
#include <cassert>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <map>
#include <vector>

namespace {
struct Port { std::vector<uint8_t> incoming, output; bool closed = false, disconnected = false; };
std::map<int, Port> ports;
int next_fd = 42, closed = 0, lock_error = 0, settings_error = 0, write_error = 0, read_error = 0;
bool zero_write = false;
}
int open(const char*, int) { ports[next_fd] = Port{}; return next_fd++; }
int close(int fd) { ports[fd].closed = true; ++closed; return 0; }
int flock(int, int) { if (lock_error) { errno = lock_error; return -1; } return 0; }
int tcgetattr(int, termios*) { if (settings_error) { errno = settings_error; return -1; } return 0; }
void cfmakeraw(termios*) {}
int cfsetispeed(termios*, unsigned) { return 0; }
int cfsetospeed(termios*, unsigned) { return 0; }
int tcsetattr(int, int, const termios*) { return 0; }
ssize_t write(int fd, const void* data, std::size_t size) {
    if (write_error) { errno = write_error; write_error = 0; return -1; }
    if (zero_write) return 0;
    auto count = std::min(size, std::size_t{3}); auto bytes = static_cast<const uint8_t*>(data);
    ports[fd].output.insert(ports[fd].output.end(), bytes, bytes + count);
    return static_cast<ssize_t>(count);
}
ssize_t read(int fd, void* data, std::size_t size) {
    if (read_error) { errno = read_error; read_error = 0; return -1; }
    // Fragment every serial packet into at most two-byte reads.
    auto& incoming = ports[fd].incoming;
    auto count = std::min(incoming.size(), std::min(size, std::size_t{2}));
    if (!count) { errno = EAGAIN; return -1; }
    std::memcpy(data, incoming.data(), count); incoming.erase(incoming.begin(), incoming.begin() + count);
    return static_cast<ssize_t>(count);
}
int poll(pollfd* request, unsigned long, int) {
    if (ports[request->fd].disconnected) { request->revents = POLLHUP; return 1; }
    request->revents = request->events;
    return request->events == POLLOUT || !ports[request->fd].incoming.empty() ? 1 : 0;
}

int main() {
    int checks = 0;
    assert(openUSBCAN(nullptr) == -1 && errno == EINVAL); ++checks;
    lock_error = EWOULDBLOCK;
    assert(openUSBCAN("busy") == -1 && errno == EWOULDBLOCK && closed == 1); ++checks;
    lock_error = 0; settings_error = ENOTTY;
    assert(openUSBCAN("not-serial") == -1 && errno == ENOTTY && closed == 2); ++checks;
    settings_error = 0;
    int fd = openUSBCAN("fake"); assert(fd >= 0); ++checks;
    FrameInfo info{0x1200FD01, EXTENDED, 8}; uint8_t data[8]{1,2,3,4,5,6,7,8};
    write_error = EINTR;
    assert(sendUSBCAN(fd, 2, &info, data) == 17 && ports[fd].output.size() == 17); ++checks;
    assert(ports[fd].output[0] == 0xA8 && ports[fd].output[1] == 2 && ports[fd].output[2] == 1);
    assert(lingzu::usb_crc8(ports[fd].output.data(), 16) == ports[fd].output[16]); ++checks;
    ports[fd].output.clear(); write_error = EAGAIN;
    assert(sendUSBCAN(fd, 1, &info, data) == 17); ++checks;
    zero_write = true; assert(sendUSBCAN(fd, 1, &info, data) == -1 && errno == EIO); ++checks;
    zero_write = false; write_error = EIO;
    assert(sendUSBCAN(fd, 1, &info, data) == -1 && errno == EIO); ++checks;
    info.dataLength = 9; assert(sendUSBCAN(fd, 1, &info, data) == -1 && errno == EINVAL); ++checks;
    info.dataLength = 8; info.frameType = 2; assert(sendUSBCAN(fd, 1, &info, data) == -1); ++checks;
    info.frameType = STANDARD; assert(sendUSBCAN(fd, 1, &info, data) == -1); ++checks;
    info.frameType = EXTENDED; assert(sendUSBCAN(fd, 3, &info, data) == -1); ++checks;
    assert(sendUSBCAN(fd, 1, nullptr, data) == -1); ++checks;
    uint8_t channel = 0, received[8]{};
    assert(readUSBCAN(fd, &channel, &info, received, -1) == -1 && errno == EINVAL); ++checks;
    assert(readUSBCAN(fd, &channel, &info, received, 0) == -1 && errno == ETIMEDOUT); ++checks;
    auto good = ports[fd].output;
    good[0] = 0xA9; good[1] = 2; good[16] = lingzu::usb_crc8(good.data(), 16);
    auto corrupt = good; corrupt[16] ^= 1;
    ports[fd].incoming = {0x11, 0x22, 0x33};
    ports[fd].incoming.insert(ports[fd].incoming.end(), corrupt.begin(), corrupt.end());
    ports[fd].incoming.insert(ports[fd].incoming.end(), good.begin(), good.end());
    ports[fd].incoming.insert(ports[fd].incoming.end(), good.begin(), good.end());
    read_error = EINTR;
    assert(readUSBCAN(fd, &channel, &info, received, 100000) == 0 && channel == 2 && info.canID == 0x1200FD01);
    assert(info.dataLength == 8 && std::memcmp(received, data, 8) == 0); ++checks;
    assert(readUSBCAN(fd, &channel, &info, received, 100000) == 0); ++checks;
    assert(readUSBCAN(fd, &channel, &info, received, 100000) == -1); ++checks;
    ports[fd].disconnected = true;
    assert(readUSBCAN(fd, &channel, &info, received, 100000) == -1 && errno == EIO); ++checks;
    assert(closeUSBCAN(fd) == 0 && ports[fd].closed); ++checks;
    assert(closeUSBCAN(fd) == -1 && errno == EBADF); ++checks;
    assert(sendUSBCAN(fd, 1, &info, data) == -1 && errno == EBADF); ++checks;
    std::cout << checks << " C++ IO checks passed with substituted OS calls\n";
}
