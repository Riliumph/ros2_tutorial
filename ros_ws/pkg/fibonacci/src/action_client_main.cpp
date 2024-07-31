// STL
#include <iostream>
#include <memory>
// ROS2
#include <rclcpp/rclcpp.hpp>
// original
#include "fibonacci/action_client.hpp"

int
main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  pid_t pid = getpid();
  std::cout << "プロセスID: " << pid << std::endl;
  auto cli = std::make_unique<fibonacci::FibonacciActionClient>();
  auto msg = fibonacci::FibonacciActionClient::Msg::Goal();
  msg.order = 10;
  auto result = cli->send(msg);
  rclcpp::shutdown();
  return 0;
}
