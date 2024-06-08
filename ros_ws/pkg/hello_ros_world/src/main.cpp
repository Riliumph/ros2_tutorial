// ROS Client Library
#include <rclcpp/rclcpp.hpp>

int
main(int argc, char** argv)
{
  const char* node_name = "hello_ros_world_node";
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(node_name);
  RCLCPP_INFO(node->get_logger(), "Hello, ROS2 world!");

  rclcpp::shutdown();
  return 0;
}
