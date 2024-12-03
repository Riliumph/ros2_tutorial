// STL
#include <chrono>
#include <thread>

// ROS Client Library
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

void
original_spin(std::shared_ptr<rclcpp::Node> node)
{
  rclcpp::Rate rate(1s);
  while (rclcpp::ok()) {
    RCLCPP_INFO(node->get_logger(), "spining ...");
    rclcpp::spin_some(node);
    rate.sleep();
  }
}

int
main(int argc, char** argv)
{
  const char* node_name = "spin_failure";
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(node_name);
  node->create_wall_timer(
    5s, [node]() { RCLCPP_INFO(node->get_logger(), "Hello, ROS2 world!"); });
  std::thread t(original_spin, node);
  while (1) {
    RCLCPP_INFO(node->get_logger(), "waiting ...");
    std::this_thread::sleep_for(1s);
  }

  rclcpp::shutdown();
  return 0;
}
