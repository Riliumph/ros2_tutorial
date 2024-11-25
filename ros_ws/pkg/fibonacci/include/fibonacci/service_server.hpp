#ifndef FIBONACCI__SERVICE_SERVER_HPP_
#define FIBONACCI__SERVICE_SERVER_HPP_
// ROS2
#include <rclcpp/rclcpp.hpp>
// ROS2 automatically created definition
#include "fibonacci/visibility_control.h"
#include "fibonacci_msg/srv/fibonacci.hpp"

namespace fibonacci {
/// @brief N回までのFibonacci数を算出して返すサービス
/// Resultで指定された回数場のFibonacci数列を返却する
class ServiceServer : public rclcpp::Node
{
public:
  using Msg = fibonacci_msg::srv::Fibonacci;

public:
  static constexpr const char* node_name = "fibonacci_server_node";
  static constexpr const char* service_name = "fibonacci_service";

public:
  FIBONACCI_PUBLIC explicit ServiceServer(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~ServiceServer();

private:
  rclcpp::Service<Msg>::SharedPtr server;
  void execute(const std::shared_ptr<Msg::Request> request,
               std::shared_ptr<Msg::Response> response);
};

} // namespace fibonacci

#endif // FIBONACCI__SERVICE_SERVER_HPP_
