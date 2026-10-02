// Offline API substitute. Actual ament/rclcpp integration is a separate CI job.
#pragma once
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <any>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <variant>
#include <vector>

namespace rclcpp {
enum class ParameterType { PARAMETER_BOOL, PARAMETER_INTEGER, PARAMETER_DOUBLE, PARAMETER_STRING };
class ParameterValue {
    std::variant<bool, int64_t, double, std::string, std::vector<std::string>> value_;
public:
    template<class T> explicit ParameterValue(T value) : value_(std::move(value)) {}
    template<class T> const T& get() const {
        const auto value = std::get_if<T>(&value_);
        if (!value) throw std::runtime_error("Parameter type mismatch");
        return *value;
    }
};
class Parameter {
    std::string name_;
    ParameterValue value_;
public:
    template<class T> Parameter(std::string name, T value) : name_(std::move(name)), value_(std::move(value)) {}
    const std::string& get_name() const { return name_; }
    template<class T> T get_value() const { return value_.get<T>(); }
    const ParameterValue& get_parameter_value() const { return value_; }
};
class NodeOptions {
public:
    std::vector<Parameter> overrides;
    NodeOptions& parameter_overrides(std::vector<Parameter> values) { overrides = std::move(values); return *this; }
};
struct Logger {};
struct Clock {};
struct Time { operator builtin_interfaces::msg::Time() const { return {}; } };
inline Logger get_logger(const std::string&) { return {}; }
struct SensorDataQoS {
    unsigned depth = 5;
    SensorDataQoS& keep_last(unsigned value) { depth = value; return *this; }
};
struct QoS {
    unsigned depth;
    explicit QoS(unsigned value) : depth(value) {}
};
template<class M> struct Subscription { using SharedPtr = std::shared_ptr<Subscription<M>>; };
template<class M> struct Publisher {
    using SharedPtr = std::shared_ptr<Publisher<M>>;
    std::vector<M> messages;
    void publish(const M& message) { messages.push_back(message); }
};
class Node {
    std::map<std::string, Parameter> overrides_, declared_;
    std::map<std::string, bool> readonly_;
    std::function<void(sensor_msgs::msg::Joy::ConstSharedPtr)> joy_callback_;
    std::function<void(trajectory_msgs::msg::JointTrajectory::ConstSharedPtr)> trajectory_callback_;
    std::map<std::string, std::any> publishers_;
public:
    using SharedPtr = std::shared_ptr<Node>;
    unsigned subscription_depth = 0;
    explicit Node(const std::string&, const NodeOptions& options = {}) {
        for (const auto& value : options.overrides) overrides_.emplace(value.get_name(), value);
    }
    Logger get_logger() const { return {}; }
    std::shared_ptr<Clock> get_clock() const { return std::make_shared<Clock>(); }
    Time now() const { return {}; }
    template<class T> T declare_parameter(const std::string& name, T initial,
        const rcl_interfaces::msg::ParameterDescriptor& descriptor = {}) {
        if (declared_.count(name)) throw std::runtime_error("Already declared: " + name);
        auto found = overrides_.find(name);
        Parameter value = found == overrides_.end() ? Parameter(name, initial) : found->second;
        const T result = value.get_value<T>();
        declared_.emplace(name, value); readonly_[name] = descriptor.read_only;
        return result;
    }
    const ParameterValue& declare_parameter(const std::string& name, ParameterType type,
        const rcl_interfaces::msg::ParameterDescriptor& descriptor = {}) {
        const auto found = overrides_.find(name);
        if (found == overrides_.end()) throw std::runtime_error("Uninitialized parameter: " + name);
        switch (type) {
            case ParameterType::PARAMETER_BOOL: declare_parameter<bool>(name, false, descriptor); break;
            case ParameterType::PARAMETER_INTEGER: declare_parameter<int64_t>(name, 0, descriptor); break;
            case ParameterType::PARAMETER_DOUBLE: declare_parameter<double>(name, 0.0, descriptor); break;
            case ParameterType::PARAMETER_STRING: declare_parameter<std::string>(name, "", descriptor); break;
        }
        return declared_.at(name).get_parameter_value();
    }
    template<class M, class Q, class Callback> typename Subscription<M>::SharedPtr create_subscription(
        const std::string&, const Q& qos, Callback callback) {
        if constexpr (std::is_same<M, sensor_msgs::msg::Joy>::value) {
            subscription_depth = qos.depth; joy_callback_ = callback;
        } else { trajectory_callback_ = callback; }
        return std::make_shared<Subscription<M>>();
    }
    template<class M, class Q> typename Publisher<M>::SharedPtr create_publisher(const std::string& topic, const Q&) {
        auto publisher = std::make_shared<Publisher<M>>(); publishers_[topic] = publisher; return publisher;
    }
    template<class M> typename Publisher<M>::SharedPtr publisher(const std::string& topic) {
        return std::any_cast<typename Publisher<M>::SharedPtr>(publishers_.at(topic));
    }
    struct SetResult { bool successful; };
    SetResult set_parameter(const Parameter& value) {
        if (readonly_[value.get_name()]) return {false};
        declared_.insert_or_assign(value.get_name(), value); return {true};
    }
    void dispatch(sensor_msgs::msg::Joy::ConstSharedPtr message) { joy_callback_(message); }
    void dispatch(trajectory_msgs::msg::JointTrajectory::ConstSharedPtr message) { trajectory_callback_(message); }
    std::function<void()> on_spin;
};
inline bool initialized = true;
struct InitOptions {};
enum class SignalHandlerOptions { None };
inline void init(int, char**, const InitOptions& = {}, SignalHandlerOptions = SignalHandlerOptions::None) { initialized = true; }
inline void shutdown() { initialized = false; }
inline bool ok() { return initialized; }
struct WallRate {
    explicit WallRate(int) {}
    void sleep() { std::this_thread::sleep_for(std::chrono::milliseconds(5)); }
};
namespace executors {
struct SingleThreadedExecutor {
    Node::SharedPtr node;
    void add_node(const Node::SharedPtr& value) { node = value; }
    void spin_some(std::chrono::nanoseconds) { if (node->on_spin) node->on_spin(); }
};
}
template<class... T> void log(const Logger&, const char*, T&&...) {}
}
#define RCLCPP_INFO(logger, ...) rclcpp::log(logger, __VA_ARGS__)
#define RCLCPP_WARN(logger, ...) rclcpp::log(logger, __VA_ARGS__)
#define RCLCPP_ERROR(logger, ...) rclcpp::log(logger, __VA_ARGS__)
#define RCLCPP_WARN_THROTTLE(logger, clock, interval, ...) rclcpp::log(logger, __VA_ARGS__)
