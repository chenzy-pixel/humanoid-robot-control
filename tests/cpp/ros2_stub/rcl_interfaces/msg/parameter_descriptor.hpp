// Offline substitute; real ROS 2 tests use installed rcl_interfaces headers.
#pragma once
namespace rcl_interfaces { namespace msg {
struct ParameterDescriptor { bool read_only = false; };
} }
