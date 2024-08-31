#ifndef FIBONACCI__ACTION_CLIENT_HPP_
#define FIBONACCI__ACTION_CLIENT_HPP_
// STL
#include <memory>
#include <optional>
#include <ostream>
// ROS2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
// other pkg
#include "fibonacci_msg/action/operator_io.hpp"
// ROS2 automatically created definition
#include "fibonacci/visibility_control.h"

namespace fibonacci {
/// @brief N個までのFibonacci数を算出して返すサービスへ通信するクライアント
class FibonacciActionClient : public rclcpp::Node
{
public:
  using Msg = fibonacci_msg::action::Fibonacci;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Msg>;

public:
  static constexpr const char* node_name = "fibonacci_action_client_node";
  static constexpr const char* client_name = "fibonacci_action_client";

public:
  FIBONACCI_PUBLIC explicit FibonacciActionClient(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  std::optional<GoalHandle::WrappedResult> Send(Msg::Goal goal);
  void Cancel(const GoalHandle::SharedPtr& request);

private:
  rclcpp_action::Client<Msg>::SharedPtr client;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string dest_server_name;
  rclcpp_action::Client<Msg>::SendGoalOptions send_options;

private:
  GoalHandle::SharedPtr SendRequest(Msg::Goal request);
  GoalHandle::WrappedResult ReceiveResponse(GoalHandle::SharedPtr goal_handle);

private: // callback
  void SentRequest(const GoalHandle::SharedPtr& request);
  void ReceiveFeedback(GoalHandle::SharedPtr goal_handle,
                       const Msg::Feedback::ConstSharedPtr feedback);
  void ReceiveResult(const GoalHandle::WrappedResult& result);
};

} // namespace fibonacci

#endif // FIBONACCI__ACTION_CLIENT_HPP_
