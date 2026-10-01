// Minimal Linux IO substitutions for testing the real transport on Windows.
#pragma once
#include <cstddef>
#include <cstdint>
using ssize_t = std::ptrdiff_t;
constexpr int O_RDWR = 2, O_NOCTTY = 256, O_NONBLOCK = 2048, O_CLOEXEC = 524288;
constexpr int LOCK_EX = 2, LOCK_NB = 4;
constexpr unsigned CLOCAL = 2048, CREAD = 128, CRTSCTS = 0x80000000u;
constexpr unsigned VMIN = 0, VTIME = 1, B115200 = 4098, TCSANOW = 0;
constexpr short POLLIN = 1, POLLOUT = 4, POLLERR = 8, POLLHUP = 16, POLLNVAL = 32;
struct termios { unsigned c_cflag = 0; unsigned char c_cc[2]{}; };
struct pollfd { int fd; short events, revents; };
int open(const char*, int);
int close(int);
ssize_t write(int, const void*, std::size_t);
ssize_t read(int, void*, std::size_t);
int flock(int, int);
int tcgetattr(int, termios*);
void cfmakeraw(termios*);
int cfsetispeed(termios*, unsigned);
int cfsetospeed(termios*, unsigned);
int tcsetattr(int, int, const termios*);
int poll(pollfd*, unsigned long, int);
