#ifndef FIBONACCI__ACTION_SERVER_HPP_
#define FIBONACCI__ACTION_SERVER_HPP_
// STL
#include <memory>
// ROS2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
// ROS2 automatically created definition
#include "fibonacci/visibility_control.h"
#include "fibonacci_msg/action/operator_io.hpp"

namespace fibonacci {
/// @brief N回までのFibonacci数を算出して返すサービス
/// Resultで指定された回数場のFibonacci数列を返却する
/// Feedbackとして一回計算がされるごとに途中経過も
class ActionServer : public rclcpp::Node
{
public:
  using Msg = fibonacci_msg::action::Fibonacci;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Msg>;

public:
  static constexpr const char* node_name = "fibonacci_action_server_node";
  static constexpr const char* server_name = "fibonacci_action_server";

public:
  FIBONACCI_PUBLIC explicit ActionServer(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  rclcpp_action::Server<Msg>::SharedPtr server;
  void execute(const std::shared_ptr<GoalHandle> goal_handle);

private: // callback
  rclcpp_action::GoalResponse Receive(const rclcpp_action::GoalUUID& uuid,
                                      std::shared_ptr<const Msg::Goal> request);
  rclcpp_action::CancelResponse Cancel(std::shared_ptr<GoalHandle> request);
  void Accept(std::shared_ptr<GoalHandle> request);
};

} // namespace fibonacci

#endif // FIBONACCI__ACTION_SERVER_HPP_
