#pragma once
#include <trajectory_msgs/msg/joint_trajectory.hpp>
namespace control_msgs::action {
struct FollowJointTrajectory {
    struct Tolerance { std::string name; double position = 0, velocity = 0, acceleration = 0; };
    struct Goal {
        trajectory_msgs::msg::JointTrajectory trajectory;
        struct MultiDOF { std::vector<std::string> joint_names; std::vector<int> points; } multi_dof_trajectory;
        std::vector<Tolerance> path_tolerance, goal_tolerance, component_path_tolerance, component_goal_tolerance;
        builtin_interfaces::msg::Duration goal_time_tolerance;
    };
    struct Result {
        static constexpr int SUCCESSFUL = 0, INVALID_GOAL = -1, INVALID_JOINTS = -2, OLD_HEADER_TIMESTAMP = -3,
                             PATH_TOLERANCE_VIOLATED = -4, GOAL_TOLERANCE_VIOLATED = -5;
        int error_code = 0; std::string error_string;
    };
    struct Feedback {
        std_msgs::msg::Header header;
        std::vector<std::string> joint_names;
        trajectory_msgs::msg::JointTrajectoryPoint desired, actual, error;
    };
};
}
