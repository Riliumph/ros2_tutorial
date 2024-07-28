// ROS2
#include <rclcpp/rclcpp.hpp>
// original
#include "fibonacci/service_client.hpp"

int
main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto cli = std::make_unique<fibonacci::FibonacciServiceClient>();
  cli->send();
  rclcpp::shutdown();
  return 0;
}
