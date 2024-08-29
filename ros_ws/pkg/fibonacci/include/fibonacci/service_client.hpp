#ifndef FIBONACCI__SERVICE_CLIENT_HPP_
#define FIBONACCI__SERVICE_CLIENT_HPP_
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

class FibonacciServiceClient : public rclcpp::Node
{
  using Msg = fibonacci_msg::srv::Fibonacci;

public:
  static constexpr const char* node_name = "fibonacci_service_client_node";
  static constexpr const char* client_name = "fibonacci_service_client";

  FIBONACCI_PUBLIC explicit FibonacciServiceClient(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  void send();

private:
  rclcpp::Client<Msg>::SharedPtr client;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string dest_server_name;
};

} // namespace fibonacci

#endif // FIBONACCI__SERVICE_CLIENT_HPP_
