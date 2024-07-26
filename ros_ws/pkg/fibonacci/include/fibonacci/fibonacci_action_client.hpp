#ifndef FIBONACCI__FIBONACCI_ACTION_CLIENT_HPP_
#define FIBONACCI__FIBONACCI_ACTION_CLIENT_HPP_
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

class FibonacciActionClient : public rclcpp::Node
{
  using ActMsg = fibonacci_msg::action::Fibonacci;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActMsg>;

public:
  static constexpr const char* node_name = "fibonacci_action_client_node";
  static constexpr const char* client_name = "fibonacci_action_client";

  FIBONACCI_PUBLIC explicit FibonacciActionClient(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  void send_goal();

private:
  rclcpp_action::Client<ActMsg>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string dest_server_name;
};

} // namespace fibonacci

#endif // FIBONACCI__FIBONACCI_ACTION_CLIENT_HPP_
