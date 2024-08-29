#ifndef FIBONACCI__SERVICE_SERVER_HPP_
#define FIBONACCI__SERVICE_SERVER_HPP_
// STL
#include <functional>
#include <memory>
#include <thread>
// ROS2
#include <rclcpp/rclcpp.hpp>
// ROS2 automatically created definition
#include "fibonacci/visibility_control.h"
#include "fibonacci_msg/srv/fibonacci.hpp"

namespace fibonacci {

class FibonacciServiceServer : public rclcpp::Node
{
  using Msg = fibonacci_msg::srv::Fibonacci;

public:
  static constexpr const char* node_name = "fibonacci_service_server_node";
  static constexpr const char* server_name = "fibonacci_service_server";

  FIBONACCI_PUBLIC explicit FibonacciServiceServer(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  rclcpp::Service<Msg>::SharedPtr server;
  void execute(const std::shared_ptr<Msg::Request> request,
               std::shared_ptr<Msg::Response> response);
};

} // namespace fibonacci

#endif // FIBONACCI__SERVICE_SERVER_HPP_
