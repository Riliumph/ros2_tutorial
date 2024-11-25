#include "greet/publisher.hpp"

namespace greet {
/// @brief コンストラクタ
/// @param options ROS2のノード設定
Publisher::Publisher(const rclcpp::NodeOptions& options)
  : Node(node_name, options)
  , count_(0)
{
  publisher_ = this->create_publisher<std_msgs::msg::String>(
    topic_name, rclcpp::QoS(buffer_size));

  timer_ = this->create_wall_timer(500ms, [this]() {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: " << message.data);
    this->publisher_->publish(message);
  });

  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " created");
}

/// @brief
Publisher::~Publisher()
{
  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " finalized");
}
} // namespace greet

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(greet::Publisher)
