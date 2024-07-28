#include "fibonacci/action_client.hpp"
// STL
#include <sstream>
// original
#include "fibonacci/action_server.hpp"

namespace fibonacci {

/// @brief コンストラクタ
/// @param options ROS2のノード設定
FibonacciActionClient::FibonacciActionClient(const rclcpp::NodeOptions& options)
  : Node(node_name, options)
  , dest_server_name(FibonacciActionServer::server_name)
{
  RCLCPP_DEBUG(this->get_logger(), "Establish Client");
  client_ptr_ = rclcpp_action::create_client<Msg>(this, dest_server_name);

  auto timer_callback_lambda = [this]() { return this->send(); };
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                   timer_callback_lambda);
}

/// @brief 実行関数
void
FibonacciActionClient::send()
{
  this->timer_->cancel();

  if (!client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Action server not available after waiting");
    rclcpp::shutdown();
  }

  auto goal_msg = Msg::Goal();
  goal_msg.order = 10;

  RCLCPP_INFO(this->get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<Msg>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this](const GoalHandle::SharedPtr& goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(),
                    "Goal accepted by server, waiting for result");
      }
    };

  send_goal_options.feedback_callback =
    [this](GoalHandle::SharedPtr,
           const std::shared_ptr<const Msg::Feedback> feedback) {
      std::stringstream ss;
      ss << "Next number in sequence received: ";
      for (auto number : feedback->partial_sequence) {
        ss << number << " ";
      }
      RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    };

  send_goal_options.result_callback =
    [this](const GoalHandle::WrappedResult& result) {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
          return;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code");
          return;
      }
      std::stringstream ss;
      ss << "Result received: ";
      for (auto number : result.result->sequence) {
        ss << number << " ";
      }
      RCLCPP_INFO(this->get_logger(), ss.str().c_str());
      rclcpp::shutdown();
    };
  this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
};

} // namespace fibonacci

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fibonacci::FibonacciActionClient)
