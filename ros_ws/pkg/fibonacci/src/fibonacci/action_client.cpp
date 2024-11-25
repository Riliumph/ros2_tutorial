#include "fibonacci/action_client.hpp"
// STL
#include <functional>
#include <sstream>
// original
#include "fibonacci/action_server.hpp"

namespace fibonacci {
/// @brief コンストラクタ
/// @param options ROS2のノード設定
ActionClient::ActionClient(const rclcpp::NodeOptions& options)
  : Node(node_name, options)
  , dest_service_name(ActionServer::service_name)
{
  using namespace std::placeholders;
  RCLCPP_DEBUG(this->get_logger(), "Establish Client");
  client = rclcpp_action::create_client<Msg>(this, dest_service_name);
  send_options.feedback_callback =
    std::bind(&ActionClient::ReceiveFeedback, this, _1, _2);
#ifdef ENABLE_CALLBACK
  send_options.goal_response_callback =
    std::bind(&ActionClient::SentRequest, this, _1);
  send_options.result_callback =
    std::bind(&ActionClient::ReceiveResult, this, _1);
#endif
}

/// @brief サーバーへ送信する関数
/// @param request 送信するゴール情報
/// @return レスポンス情報
std::optional<ActionClient::GoalHandle::WrappedResult>
ActionClient::Send(Msg::Goal request)
{
  if (!client->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Action server not available after waiting");
  }
  // コールバックのSentRequest()に相当する処理
  auto goal_handle = SendRequest(request);
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Request was rejected");
    return std::nullopt;
  }

  // コールバックのReceiveResult()に相当する処理
  auto response = ReceiveResponse(goal_handle);
  return response;
}

/// @brief アクションサーバーに処理の中止を要求する
/// @param request 中止するリクエストを特定する情報
void
ActionClient::Cancel(const GoalHandle::SharedPtr& request)
{
  client->async_cancel_goal(request);
  // auto result_future = client->async_get_result(goal_handle);
  // のfuture-blockingが解除される
}

/// @brief リクエストの送信を行う
/// @param request サーバーへ送信する情報
/// @return 受信時に使うハンドル（ID）
ActionClient::GoalHandle::SharedPtr
ActionClient::SendRequest(Msg::Goal request)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Sending request: " << request);
  auto goal_handle_future = client->async_send_goal(request, send_options);
  RCLCPP_INFO(this->get_logger(), "Waiting for accept");
  auto accepted = rclcpp::spin_until_future_complete(
    this->get_node_base_interface(), goal_handle_future);
  switch (accepted) {
    case rclcpp::FutureReturnCode::SUCCESS:
      RCLCPP_INFO(this->get_logger(), "Request was accepted");
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
  return goal_handle_future.get();
}

/// @brief レスポンスを受信する処理
/// @param goal_handle サーバーにリクエストを特定させるためのハンドル（ID）
/// @return
ActionClient::GoalHandle::WrappedResult
ActionClient::ReceiveResponse(GoalHandle::SharedPtr goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Request result");
  auto result_future = client->async_get_result(goal_handle);
  RCLCPP_INFO(this->get_logger(), "Waiting for result");
  auto response = rclcpp::spin_until_future_complete(
    this->get_node_base_interface(), result_future);
  switch (response) {
    case rclcpp::FutureReturnCode::SUCCESS:
      RCLCPP_INFO(this->get_logger(), "Request was succeeded");
      break;
    case rclcpp::FutureReturnCode::INTERRUPTED:
      RCLCPP_ERROR(this->get_logger(), "Request was interrupted");
      break;
    case rclcpp::FutureReturnCode::TIMEOUT:
      RCLCPP_ERROR(this->get_logger(), "Request was timeout");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Request was missed");
      break;
  }
  return result_future.get();
}

/// @brief サーバーからのACCEPTの受信時に発火するコールバック
/// サーバーのReceiveコールバックの戻り値を引数で受ける。
/// SendRequest()の後半部分に相当する処理である。
/// rclcpp::spin_until_future_complete()は内部で実行される。
/// そのためコールバックによる実装では以下の処理は不要であり、不可能である。
/// - rclcpp::FutureReturnCodeなどのチェック
/// @param response サーバーのReceiveの結果を格納したデータ
void
ActionClient::SentRequest(const GoalHandle::SharedPtr& response)
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
ActionClient::ReceiveFeedback(GoalHandle::SharedPtr goal_handle,
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
/// ReceiveResponse()の後半部分に相当する処理である。
/// rclcpp::spin_until_future_complete()は内部で実行される。
/// そのためコールバックによる実装では以下の処理は不要であり、不可能である。
/// - rclcpp::FutureReturnCodeなどのチェック
/// @param result リザルト
void
ActionClient::ReceiveResult(const GoalHandle::WrappedResult& result)
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
RCLCPP_COMPONENTS_REGISTER_NODE(fibonacci::ActionClient)
