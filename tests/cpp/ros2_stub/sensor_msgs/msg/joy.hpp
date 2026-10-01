// Offline substitute; does not test ROS message serialization or DDS.
#pragma once
#include <cstdint>
#include <memory>
#include <vector>
namespace sensor_msgs { namespace msg {
struct Joy {
    using ConstSharedPtr = std::shared_ptr<const Joy>;
    std::vector<float> axes;
    std::vector<int32_t> buttons;
};
} }
