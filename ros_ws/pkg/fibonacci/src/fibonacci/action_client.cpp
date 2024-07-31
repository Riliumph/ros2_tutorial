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

  // auto timer_callback_lambda = [this]() {
  //   auto goal = Msg::Goal();
  //   goal.order = 10;
  //   return this->send(goal);
  // };
  // timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
  //                                  timer_callback_lambda);
}

/// @brief 実行関数
FibonacciActionClient::Msg::Result::SharedPtr
FibonacciActionClient::send(Msg::Goal goal)
{
  // this->timer_->cancel();

  if (!client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Action server not available after waiting");
    rclcpp::shutdown(); // rclcpp::spinの解除
  }

  RCLCPP_INFO(this->get_logger(), "Sending goal");

  auto send_options = rclcpp_action::Client<Msg>::SendGoalOptions();
  send_options.feedback_callback =
    [this](GoalHandle::SharedPtr,
           const std::shared_ptr<const Msg::Feedback> feedback) {
      std::stringstream ss;
      ss << "Next number in sequence received: ";
      for (auto number : feedback->partial_sequence) {
        ss << number << " ";
      }
      RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    };
  // アクセプト待ち
  auto goal_handle_future =
    this->client_ptr_->async_send_goal(goal, send_options);
  auto accepted = rclcpp::spin_until_future_complete(
    this->get_node_base_interface(), goal_handle_future);
  if (accepted != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "failed to call send_goal");
    return nullptr;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected");
    return nullptr;
  }

  RCLCPP_INFO(this->get_logger(), "wait for result from server");
  auto result_future = client_ptr_->async_get_result(goal_handle);
  auto response = rclcpp::spin_until_future_complete(
    this->get_node_base_interface(), result_future);
  if (response != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "failed to get result");
    return nullptr;
  }
  auto result = result_future.get();
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "goal was aborted");
      return nullptr;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "goal was canceled");
      return nullptr;
    default:
      RCLCPP_ERROR(this->get_logger(),
                   "unexpected return code(%d)",
                   static_cast<int8_t>(result.code));
      return nullptr;
  }
  return result.result;
}

} // namespace fibonacci

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fibonacci::FibonacciActionClient)
