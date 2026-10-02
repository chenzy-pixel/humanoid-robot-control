#pragma once
#include <builtin_interfaces/msg/time.hpp>
#include <string>
namespace std_msgs::msg {
struct Header { builtin_interfaces::msg::Time stamp; std::string frame_id; };
}
