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

/// @brief サーバーへ送信する関数
/// @param request 送信するゴール情報
/// @return レスポンス情報
FibonacciActionClient::Msg::Result::SharedPtr
FibonacciActionClient::send(Msg::Goal request)
{
  if (!client->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Action server not available after waiting");
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "Sending request: " << request);
  auto goal_handle_future =
    this->client->async_send_goal(request, send_options);

  RCLCPP_INFO(this->get_logger(), "Waiting for accept");
  auto accepted = rclcpp::spin_until_future_complete(
    this->get_node_base_interface(), goal_handle_future);
  switch (accepted) {
    case rclcpp::FutureReturnCode::SUCCESS:
      RCLCPP_INFO(this->get_logger(), "request was accepted");
      break;
    case rclcpp::FutureReturnCode::INTERRUPTED:
      RCLCPP_ERROR(this->get_logger(), "Request was interrupted");
      return nullptr;
    case rclcpp::FutureReturnCode::TIMEOUT:
      RCLCPP_ERROR(this->get_logger(), "Request was timeout");
      return nullptr;
    default:
      RCLCPP_ERROR(this->get_logger(), "Request was missed");
      return nullptr;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Request was rejected");
    return nullptr;
  }

  RCLCPP_INFO(this->get_logger(), "Request result");
  auto result_future = client->async_get_result(goal_handle);

  RCLCPP_INFO(this->get_logger(), "Wait for result");
  auto response = rclcpp::spin_until_future_complete(
    this->get_node_base_interface(), result_future);
  switch (response) {
    case rclcpp::FutureReturnCode::SUCCESS:
      RCLCPP_INFO(this->get_logger(), "request was accepted");
      break;
    case rclcpp::FutureReturnCode::INTERRUPTED:
      RCLCPP_ERROR(this->get_logger(), "Request was interrupted");
      return nullptr;
    case rclcpp::FutureReturnCode::TIMEOUT:
      RCLCPP_ERROR(this->get_logger(), "Request was timeout");
      return nullptr;
    default:
      RCLCPP_ERROR(this->get_logger(), "Request was missed");
      return nullptr;
  }

  auto result = result_future.get();
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "request was succeeded");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "request was aborted");
      return nullptr;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "request was canceled");
      return nullptr;
    default:
      RCLCPP_ERROR(this->get_logger(),
                   "unexpected return code(%d)",
                   static_cast<int8_t>(result.code));
      return nullptr;
  }
  return result.result;
}

/// @brief アクションサーバーに処理の中止を要求する
/// @param request 中止するリクエストを特定する情報
void
FibonacciActionClient::Cancel(const GoalHandle::SharedPtr& request)
{
  client->async_cancel_goal(request);
  // auto result_future = client->async_get_result(goal_handle);
  // のfuture-blockingが解除される
}

/// @brief サーバーからのACCEPTの受信時に発火するコールバック
/// サーバーのReceiveコールバックの戻り値を引数で受ける。
/// @param response サーバーのReceiveの結果を格納したデータ
void
FibonacciActionClient::ReceiveRequest(const GoalHandle::SharedPtr& response)
{
  if (!response) {
    RCLCPP_ERROR(this->get_logger(), "Request was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "response was accepted by server");
  }
}

/// @brief フィードバック受信時に発火するコールバック
/// @param goal_handle リクエストを特定する情報
/// @param response フィードバックとして受け取るレスポンス情報
void
FibonacciActionClient::ReceiveFeedback(
  GoalHandle::SharedPtr goal_handle,
  const Msg::Feedback::ConstSharedPtr response)
{
  RCLCPP_INFO(this->get_logger(), "Received feedback");
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Next number in sequence received: " << *response);

  // 10以上は大きすぎるので必要ない
  auto max_it = std::max_element(response->partial_sequence.begin(),
                                 response->partial_sequence.end());
  if (max_it != response->partial_sequence.end()) {
    if (10 < *max_it) {
      RCLCPP_INFO(this->get_logger(),
                  "fibonacci support under 10. canceling...");
      Cancel(goal_handle);
    }
  }
}

/// @brief リザルト受信時に発火するコールバック
/// @param result リザルト
void
FibonacciActionClient::ReceiveResult(const GoalHandle::WrappedResult& result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Request was succeeded");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Request was aborted");
      rclcpp::shutdown();
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Request was canceled");
      rclcpp::shutdown();
      return;
    default:
      RCLCPP_ERROR(this->get_logger(),
                   "unexpected return code(%d)",
                   static_cast<int8_t>(result.code));
      rclcpp::shutdown();
      return;
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Result received: " << result.result);
  rclcpp::shutdown(); // rclcpp::spinの停止
}
} // namespace fibonacci

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fibonacci::FibonacciActionClient)
