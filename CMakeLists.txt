cmake_minimum_required(VERSION 3.16)
project(humanoid_robot_control_checks LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(Threads REQUIRED)
enable_testing()
set(SDK_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/ros2_ws/src/usb2can_demo_lingzu/can")
set(COMMON_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/common/include")

add_executable(protocol_test tests/cpp/protocol_test.cpp)
target_include_directories(protocol_test PRIVATE "${COMMON_INCLUDE}" "${SDK_INCLUDE}")
target_link_libraries(protocol_test PRIVATE Threads::Threads)
add_test(NAME protocol_and_transport COMMAND protocol_test)

add_executable(usb2can_io_test tests/cpp/usb2can_io_test.cpp common/src/usb_can.cpp)
target_include_directories(usb2can_io_test PRIVATE tests/cpp/linux_io_stub "${COMMON_INCLUDE}" "${SDK_INCLUDE}")
target_link_libraries(usb2can_io_test PRIVATE Threads::Threads)
add_test(NAME usb2can_simulated_serial_io COMMAND usb2can_io_test)

# Compile API usage only. Production ROS builds use scripts/build_ros2.sh.
add_library(ros2_api_compile_check OBJECT
    ros2_ws/src/usb2can_demo_lingzu/src/Tangair_usb2can.cpp
    ros2_ws/src/usb2can_demo_lingzu/src/can_node.cpp)
target_include_directories(ros2_api_compile_check PRIVATE tests/cpp/ros2_stub
    ros2_ws/src/usb2can_demo_lingzu/include "${COMMON_INCLUDE}" "${SDK_INCLUDE}")
set(CHECK_TARGETS protocol_test usb2can_io_test ros2_api_compile_check)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_library(usb2can_linux_compile_check OBJECT common/src/usb_can.cpp)
    target_include_directories(usb2can_linux_compile_check PRIVATE "${COMMON_INCLUDE}" "${SDK_INCLUDE}")
    list(APPEND CHECK_TARGETS usb2can_linux_compile_check)
endif()
foreach(target IN LISTS CHECK_TARGETS)
    target_compile_options(${target} PRIVATE -Wall -Wextra -Werror)
    # Assertions remain active even in Release test builds.
    target_compile_options(${target} PRIVATE -UNDEBUG)
endforeach()

add_executable(ros2_config_test tests/cpp/ros2_config_test.cpp ros2_ws/src/usb2can_demo_lingzu/src/Tangair_usb2can.cpp)
target_include_directories(ros2_config_test PRIVATE tests/cpp/ros2_stub ros2_ws/src/usb2can_demo_lingzu/include "${COMMON_INCLUDE}" "${SDK_INCLUDE}")
target_link_libraries(ros2_config_test PRIVATE Threads::Threads)
target_compile_options(ros2_config_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
add_test(NAME ros2_config_and_shutdown COMMAND ros2_config_test)
