set(ROBOT_RUCKIG_ROOT "${CMAKE_CURRENT_LIST_DIR}/../third_party/ruckig")
add_library(robot_ruckig STATIC
    ${ROBOT_RUCKIG_ROOT}/src/ruckig/brake.cpp
    ${ROBOT_RUCKIG_ROOT}/src/ruckig/position_first_step1.cpp
    ${ROBOT_RUCKIG_ROOT}/src/ruckig/position_first_step2.cpp
    ${ROBOT_RUCKIG_ROOT}/src/ruckig/position_second_step1.cpp
    ${ROBOT_RUCKIG_ROOT}/src/ruckig/position_second_step2.cpp
    ${ROBOT_RUCKIG_ROOT}/src/ruckig/position_third_step1.cpp
    ${ROBOT_RUCKIG_ROOT}/src/ruckig/position_third_step2.cpp
    ${ROBOT_RUCKIG_ROOT}/src/ruckig/velocity_second_step1.cpp
    ${ROBOT_RUCKIG_ROOT}/src/ruckig/velocity_second_step2.cpp
    ${ROBOT_RUCKIG_ROOT}/src/ruckig/velocity_third_step1.cpp
    ${ROBOT_RUCKIG_ROOT}/src/ruckig/velocity_third_step2.cpp)
target_compile_features(robot_ruckig PUBLIC cxx_std_17)
target_include_directories(robot_ruckig SYSTEM PUBLIC "${ROBOT_RUCKIG_ROOT}/include")
if(WIN32)
    target_compile_definitions(robot_ruckig PUBLIC _USE_MATH_DEFINES)
endif()
