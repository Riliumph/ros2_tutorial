#ifndef FIBONACCI__SERVICE_CLIENT_HPP_
#define FIBONACCI__SERVICE_CLIENT_HPP_
// STL
#include <functional>
#include <memory>
#include <thread>
// ROS2
#include <rclcpp/rclcpp.hpp>
// other pkg
#include "fibonacci_msg/srv/operator_io.hpp"
// ROS2 automatically created definition
#include "fibonacci/visibility_control.h"

namespace fibonacci {
/// @brief Fibonacci数を算出して返すサービスサーバーへ通信するクライアント
class ServiceClient : public rclcpp::Node
{
public:
  using Msg = fibonacci_msg::srv::Fibonacci;

public:
  static constexpr const char* node_name = "fibonacci_service_client_node";
  static constexpr const char* client_name = "fibonacci_service_client";

public:
  FIBONACCI_PUBLIC explicit ServiceClient(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~ServiceClient();

  void send();

private:
  rclcpp::Client<Msg>::SharedPtr client;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string dest_service_name;
};

} // namespace fibonacci

#endif // FIBONACCI__SERVICE_CLIENT_HPP_
