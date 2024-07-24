#ifndef FIBONACCI__FIBONACCI_ACTION_SERVER_HPP_
#define FIBONACCI__FIBONACCI_ACTION_SERVER_HPP_
// STL
#include <functional>
#include <memory>
#include <thread>
// ROS2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
// ROS2 automatically created definition
#include "fibonacci/visibility_control.h"
#include "fibonacci_msg/action/fibonacci.hpp"

namespace fibonacci {

class FibonacciActionServer : public rclcpp::Node
{
  using ActMsg = fibonacci_msg::action::Fibonacci;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActMsg>;

public:
  static const char* node_name;
  static const char* server_name;

  FIBONACCI_PUBLIC explicit FibonacciActionServer(
    const rclcpp::NodeOptions& options);

  virtual ~FibonacciActionServer();

private:
  rclcpp_action::Server<ActMsg>::SharedPtr action_server_;

  void execute(const std::shared_ptr<GoalHandle> goal_handle);
};

} // namespace fibonacci

#endif // FIBONACCI__FIBONACCI_ACTION_SERVER_HPP_
