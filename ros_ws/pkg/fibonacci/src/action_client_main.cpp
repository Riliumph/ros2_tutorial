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
  auto cli = std::make_shared<fibonacci::FibonacciActionClient>();
  auto goal = fibonacci::FibonacciActionClient::Msg::Goal();
  goal.order = 10;
  auto result = cli->send(goal);
  std::stringstream ss;
  ss << "Received: ";
  for (const auto& s : result->sequence) {
    ss << s << " ";
  }
  std::cout << ss.str() << std::endl;
  rclcpp::shutdown();
  return 0;
}
