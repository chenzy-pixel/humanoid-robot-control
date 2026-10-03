cmake_minimum_required(VERSION 3.16)
project(humanoid_robot_control_checks LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(Threads REQUIRED)
enable_testing()
set(SDK_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/ros2_ws/src/usb2can_demo_lingzu/can")
set(COMMON_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/common/include")
set(CONTROLLER_SOURCES
    ros2_ws/src/usb2can_demo_lingzu/src/Tangair_usb2can.cpp
    ros2_ws/src/usb2can_demo_lingzu/src/controller_api.cpp
    ros2_ws/src/usb2can_demo_lingzu/src/controller_motion.cpp)
include(common/ruckig.cmake)
add_library(joint_motion STATIC common/src/joint_motion.cpp)
target_include_directories(joint_motion PUBLIC "${COMMON_INCLUDE}")
target_link_libraries(joint_motion PUBLIC robot_ruckig)
target_compile_options(joint_motion PRIVATE -Wall -Wextra -Werror -UNDEBUG)
add_executable(trajectory_test tests/cpp/trajectory_test.cpp)
target_link_libraries(trajectory_test PRIVATE joint_motion)
target_compile_options(trajectory_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
add_test(NAME joint_trajectory_limits COMMAND trajectory_test)

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
    ${CONTROLLER_SOURCES}
    ros2_ws/src/usb2can_demo_lingzu/src/can_node.cpp)
target_include_directories(ros2_api_compile_check PRIVATE tests/cpp/ros2_stub
    ros2_ws/src/usb2can_demo_lingzu/include "${COMMON_INCLUDE}" "${SDK_INCLUDE}")
target_link_libraries(ros2_api_compile_check PRIVATE joint_motion)
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

add_executable(ros2_config_test tests/cpp/ros2_config_test.cpp ${CONTROLLER_SOURCES})
target_include_directories(ros2_config_test PRIVATE tests/cpp/ros2_stub ros2_ws/src/usb2can_demo_lingzu/include "${COMMON_INCLUDE}" "${SDK_INCLUDE}")
target_link_libraries(ros2_config_test PRIVATE Threads::Threads joint_motion)
target_compile_options(ros2_config_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
add_test(NAME ros2_config_and_shutdown COMMAND ros2_config_test)
add_executable(ros2_motion_test tests/cpp/ros2_motion_test.cpp ${CONTROLLER_SOURCES})
target_include_directories(ros2_motion_test PRIVATE tests/cpp/ros2_stub ros2_ws/src/usb2can_demo_lingzu/include "${COMMON_INCLUDE}" "${SDK_INCLUDE}")
target_link_libraries(ros2_motion_test PRIVATE Threads::Threads joint_motion)
target_compile_options(ros2_motion_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
add_test(NAME ros2_motion_execution COMMAND ros2_motion_test)
