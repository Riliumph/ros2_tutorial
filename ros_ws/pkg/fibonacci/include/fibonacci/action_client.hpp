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
class ActionClient : public rclcpp::Node
{
public:
  using Msg = fibonacci_msg::action::Fibonacci;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Msg>;

public:
  static constexpr const char* node_name = "fibonacci_action_client_node";
  static constexpr const char* client_name = "fibonacci_action_client";

public:
  FIBONACCI_PUBLIC explicit ActionClient(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~ActionClient();

  std::optional<GoalHandle::WrappedResult> Send(Msg::Goal);
  void Cancel(const GoalHandle::SharedPtr&);

private:
  rclcpp_action::Client<Msg>::SharedPtr client;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string dest_service_name;
  rclcpp_action::Client<Msg>::SendGoalOptions send_options;

private:
  GoalHandle::SharedPtr SendRequest(Msg::Goal);
  GoalHandle::WrappedResult ReceiveResponse(GoalHandle::SharedPtr);

private: // callback
  void SentRequest(const GoalHandle::SharedPtr&);
  void ReceiveFeedback(GoalHandle::SharedPtr, Msg::Feedback::ConstSharedPtr);
  void ReceiveResult(const GoalHandle::WrappedResult&);
};

} // namespace fibonacci

#endif // FIBONACCI__ACTION_CLIENT_HPP_
