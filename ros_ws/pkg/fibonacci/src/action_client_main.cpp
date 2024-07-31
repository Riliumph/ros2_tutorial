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
  rclcpp::spin(cli); // callback event生成
  rclcpp::shutdown();
  return 0;
}
