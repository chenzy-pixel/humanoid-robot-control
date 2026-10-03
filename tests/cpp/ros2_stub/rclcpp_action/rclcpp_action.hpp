// Behavior substitute for offline tests. Native ROS action integration has its own test.
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <array>
namespace rclcpp_action {
using GoalUUID = std::array<uint8_t, 16>;
enum class GoalResponse { REJECT, ACCEPT_AND_EXECUTE };
enum class CancelResponse { REJECT, ACCEPT };
template<class A> struct ServerGoalHandle {
    std::shared_ptr<const typename A::Goal> goal;
    GoalUUID id{};
    std::string state = "executing";
    std::shared_ptr<typename A::Result> result;
    std::vector<typename A::Feedback> feedback;
    const auto& get_goal() const { return goal; }
    const GoalUUID& get_goal_id() const { return id; }
    bool is_canceling() const { return state == "canceling"; }
    bool is_active() const { return state == "executing" || is_canceling(); }
    void succeed(std::shared_ptr<typename A::Result> r) { assert_active(); state = "succeeded"; result = r; }
    void abort(std::shared_ptr<typename A::Result> r) { assert_active(); state = "aborted"; result = r; }
    void canceled(std::shared_ptr<typename A::Result> r) { if (!is_canceling()) throw std::runtime_error("Goal is not canceling"); state = "canceled"; result = r; }
    void publish_feedback(std::shared_ptr<typename A::Feedback> f) { if (!is_active()) throw std::runtime_error("Inactive feedback"); feedback.push_back(*f); }
    void assert_active() const { if (!is_active()) throw std::runtime_error("Goal already terminal"); }
};
template<class A> struct Server {
    using SharedPtr = std::shared_ptr<Server<A>>;
    using Handle = ServerGoalHandle<A>;
    std::function<GoalResponse(const GoalUUID&, std::shared_ptr<const typename A::Goal>)> goal_callback;
    std::function<CancelResponse(std::shared_ptr<Handle>)> cancel_callback;
    std::function<void(std::shared_ptr<Handle>)> accepted_callback;
    unsigned sequence = 0;
    std::shared_ptr<Handle> send_goal(const typename A::Goal& goal) {
        auto h = std::make_shared<Handle>(); h->goal = std::make_shared<typename A::Goal>(goal); h->id[0] = ++sequence;
        if (goal_callback(h->id, h->goal) == GoalResponse::REJECT) return {};
        accepted_callback(h); return h;
    }
    bool cancel_goal(std::shared_ptr<Handle> h) {
        if (cancel_callback(h) != CancelResponse::ACCEPT) return false;
        h->state = "canceling"; return true;
    }
};
template<class A, class Node, class G, class C, class H>
typename Server<A>::SharedPtr create_server(Node node, const std::string& name, G goal, C cancel, H accepted) {
    auto server = std::make_shared<Server<A>>(); server->goal_callback = goal; server->cancel_callback = cancel; server->accepted_callback = accepted;
    node->attach_interface(name, server); return server;
}
}
