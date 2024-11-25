#ifndef GREET__SUBSCRIBER_HPP_
#define GREET__SUBSCRIBER_HPP_
// STL
#include <chrono>
#include <memory>
#include <string>
// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
// ROS2 automatically created definition
#include "greet/visibility_control.h"

using namespace std::chrono_literals;
namespace greet {
/// @brief パブリッシャー
class Subscriber : public rclcpp::Node
{
public:
  static constexpr const char* node_name = "greet_subscriber_node";
  static constexpr const size_t buffer_size = 10;

public:
  GREET_PUBLIC explicit Subscriber(const rclcpp::NodeOptions& options);
  ~Subscriber();

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
}
#endif // GREET__SUBSCRIBER_HPP_
