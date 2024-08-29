#ifndef FIBONACCI__ACTION_SERVER_HPP_
#define FIBONACCI__ACTION_SERVER_HPP_
// STL
#include <functional>
#include <memory>
#include <thread>
// ROS2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
// ROS2 automatically created definition
#include "fibonacci/visibility_control.h"
#include "fibonacci_msg/action/fibonacci.hpp"

namespace fibonacci {

class FibonacciActionServer : public rclcpp::Node
{
  using Msg = fibonacci_msg::action::Fibonacci;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Msg>;

public:
  static constexpr const char* node_name = "fibonacci_action_server_node";
  static constexpr const char* server_name = "fibonacci_action_server";

  FIBONACCI_PUBLIC explicit FibonacciActionServer(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  rclcpp_action::Server<Msg>::SharedPtr server;
  void execute(const std::shared_ptr<GoalHandle> goal_handle);
};

} // namespace fibonacci

#endif // FIBONACCI__ACTION_SERVER_HPP_
