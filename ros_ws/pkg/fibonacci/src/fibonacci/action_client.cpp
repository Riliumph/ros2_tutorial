#include "fibonacci/action_client.hpp"
// STL
#include <functional>
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
  using namespace std::placeholders;
  RCLCPP_DEBUG(this->get_logger(), "Establish Client");
  client = rclcpp_action::create_client<Msg>(this, dest_server_name);
  send_options.feedback_callback =
    std::bind(&FibonacciActionClient::ReceiveFeedback, this, _1, _2);
  // send_options.goal_response_callback =
  //   std::bind(&FibonacciActionClient::ReceiveRequest, this, _1);
  // send_options.result_callback =
  //   std::bind(&FibonacciActionClient::ReceiveResult, this, _1);
}

/// @brief 実行関数
FibonacciActionClient::Msg::Result::SharedPtr
FibonacciActionClient::send(Msg::Goal goal)
{
  // this->timer_->cancel();

  if (!client->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Action server not available after waiting");
    rclcpp::shutdown(); // rclcpp::spinの解除
  }

  RCLCPP_INFO(this->get_logger(), "Sending goal");

  // アクセプト待ち
  auto goal_handle_future = this->client->async_send_goal(goal, send_options);
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
  auto result_future = client->async_get_result(goal_handle);
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

void
FibonacciActionClient::Cancel(const GoalHandle::SharedPtr& request)
{
  client->async_cancel_goal(request);
  // auto result_future = client->async_get_result(goal_handle);
  // のfuture-blockingが解除される
}

/// @brief リクエスト受信時にコールバックされる関数
/// @param request クライアントからのゴールハンドル
void
FibonacciActionClient::ReceiveRequest(const GoalHandle::SharedPtr& request)
{
  if (!request) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Goal accepted by server, waiting for result");
  }
}

/// @brief フィードバック受信時にコールバックされる関数
/// @param goal_handle ゴールハンドル
/// @param feedback フィードバックデータ
void
FibonacciActionClient::ReceiveFeedback(
  GoalHandle::SharedPtr goal_handle,
  const Msg::Feedback::ConstSharedPtr feedback)
{
  RCLCPP_INFO(this->get_logger(), "Received feedback");
  std::stringstream ss;
  ss << "Next number in sequence received: ";
  for (auto number : feedback->partial_sequence) {
    ss << number << " ";
  }
  RCLCPP_INFO(this->get_logger(), ss.str().data());

  // 10以上は大きすぎるので必要ない
  auto max_it = std::max_element(feedback->partial_sequence.begin(),
                                 feedback->partial_sequence.end());
  if (max_it != feedback->partial_sequence.end()) {
    if (10 < *max_it) {
      RCLCPP_INFO(this->get_logger(),
                  "fibonacci support under 10. canceling...");
      Cancel(goal_handle);
    }
  }
}

/// @brief リザルト受信時にコールバックされる関数
/// @param result リザルト
void
FibonacciActionClient::ReceiveResult(const GoalHandle::WrappedResult& result)
{
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
  rclcpp::shutdown(); // rclcpp::spinの停止
}

} // namespace fibonacci

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fibonacci::FibonacciActionClient)
