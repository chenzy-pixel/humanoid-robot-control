#pragma once
#include <cstdint>
namespace builtin_interfaces::msg {
struct Time { int32_t sec = 0; uint32_t nanosec = 0; };
struct Duration { int32_t sec = 0; uint32_t nanosec = 0; };
}
