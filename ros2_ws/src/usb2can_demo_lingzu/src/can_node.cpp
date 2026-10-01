// Copyright (c) 2023-2025 TANGAIR
// SPDX-License-Identifier: Apache-2.0
#include "Tangair_usb2can.h"
#include <exception>
namespace {
static_assert(ATOMIC_BOOL_LOCK_FREE == 2, "Signal stop flag must always be lock-free");
std::atomic<bool> stop_requested{false};
void request_stop(int) { stop_requested.store(true, std::memory_order_relaxed); }
}
int main(int argc, char** argv) {
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, request_stop);
    std::signal(SIGTERM, request_stop);
    int result = 0;
    try {
        auto node = std::make_shared<rclcpp::Node>("can_motor_controller_node");
        Tangair_usb2can controller(node);
        controller.Spin(&stop_requested);
    } catch (const std::exception& error) {
        RCLCPP_ERROR(rclcpp::get_logger("can_motor_controller_node"), "Controller stopped: %s", error.what());
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
