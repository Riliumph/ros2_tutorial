#include "fibonacci/service_client.hpp"
// original
#include "fibonacci/service_server.hpp"

namespace fibonacci {
/// @brief コンストラクタ
/// @param options ROS2のノード設定
ServiceClient::ServiceClient(const rclcpp::NodeOptions& options)
  : Node(node_name, options)
  , dest_service_name(ServiceServer::service_name)
{
  RCLCPP_DEBUG(get_logger(), "Establish Service Client");
  client = create_client<Msg>(dest_service_name);

  auto timer_callback_lambda = [this]() { return send(); };
  timer_ =
    create_wall_timer(std::chrono::milliseconds(500), timer_callback_lambda);
  RCLCPP_INFO_STREAM(get_logger(), get_name() << " created");
}

/// @brief デストラクタ
ServiceClient::~ServiceClient()
{
  RCLCPP_INFO_STREAM(get_logger(), get_name() << " finalized");
}

/// @brief サーバーへ送信する関数
/// 現状、外部から値を受ける仕組みはない。
/// @return レスポンス情報
void
ServiceClient::send()
{
  timer_->cancel();

  if (!client->wait_for_service()) {
    RCLCPP_ERROR(get_logger(), "Service server not available after waiting");
    rclcpp::shutdown();
  }

  auto request = std::make_shared<Msg::Request>();
  request->order = 10;

  RCLCPP_INFO(get_logger(), "Sending request");
#ifdef ENABLE_CALLBACK
  // async_send_request内で以下のことが実行されているため実装不要
  // rclcpp::FutureReturnCode::SUCCESSなどのチェック
  auto result = client->async_send_request(request, [this](auto future_result) {
    auto result = future_result.get();
    RCLCPP_INFO_STREAM(get_logger(), "Result received: " << *result);
  });
#else
  auto future_result = client->async_send_request(request);
  RCLCPP_INFO(get_logger(), "Waiting for response");
  // TODO: Segmentation faultの発生を確認
  // Attention: rclcpp::spinとの多重spin発生に気を付ける
  auto return_code = rclcpp::spin_until_future_complete(
    get_node_base_interface(), future_result);
  switch (return_code) {
    case rclcpp::FutureReturnCode::SUCCESS:
      RCLCPP_INFO(get_logger(), "Request was succeeded");
      break;
    case rclcpp::FutureReturnCode::INTERRUPTED:
      RCLCPP_ERROR(get_logger(), "Request was interrupted");
      return;
    case rclcpp::FutureReturnCode::TIMEOUT:
      RCLCPP_ERROR(get_logger(), "Request was timeout");
      return;
    default:
      RCLCPP_ERROR(get_logger(), "Request was missed");
      return;
  }
  auto result = future_result.get();
  RCLCPP_INFO_STREAM(get_logger(), "Result received: " << *result);
#endif
}
} // namespace fibonacci

// #include "rclcpp_components/register_node_macro.hpp"
// RCLCPP_COMPONENTS_REGISTER_NODE(fibonacci::ServiceClient)
