#include "greet/subscriber.hpp"
// original
#include "greet/publisher.hpp"

namespace greet {
/// @brief コンストラクタ
/// @param options ROS2のノード設定
Subscriber::Subscriber(const rclcpp::NodeOptions& options)
  : Node(node_name, options)
{
  auto topic_callback = [this](std_msgs::msg::String::UniquePtr msg) -> void {
    RCLCPP_INFO_STREAM(this->get_logger(), "I heard: " << msg->data);
  };
  subscription_ = this->create_subscription<std_msgs::msg::String>(
    Publisher::topic_name, rclcpp::QoS(buffer_size), topic_callback);

  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " created");
}

/// @brief
Subscriber::~Subscriber()
{
  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " finalized");
}
} // namespace greet

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(greet::Subscriber)
