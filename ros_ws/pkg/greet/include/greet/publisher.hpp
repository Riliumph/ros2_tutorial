#ifndef GREET__PUBLISHER_HPP_
#define GREET__PUBLISHER_HPP_
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
class Publisher : public rclcpp::Node
{
public:
  static constexpr const char* node_name = "greet_publisher_node";
  static constexpr const char* topic_name = "greet_topic";
  static constexpr const size_t buffer_size = 10;

public:
  GREET_PUBLIC explicit Publisher(const rclcpp::NodeOptions& options);
  ~Publisher();

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};
}
#endif // GREET__PUBLISHER_HPP_
