#include "fibonacci/service_client.hpp"
// STL
#include <sstream>
// original
#include "fibonacci/service_server.hpp"

namespace fibonacci {

/// @brief コンストラクタ
/// @param options ROS2のノード設定
FibonacciServiceClient::FibonacciServiceClient(
  const rclcpp::NodeOptions& options)
  : Node(node_name, options)
  , dest_server_name(FibonacciServiceServer::server_name)
{
  RCLCPP_DEBUG(this->get_logger(), "Establish Service Client");
  client_ptr_ = this->create_client<Msg>(dest_server_name);

  auto timer_callback_lambda = [this]() { return this->send(); };
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                   timer_callback_lambda);
}

/// @brief 実行関数
void
FibonacciServiceClient::send()
{
  this->timer_->cancel();

  if (!client_ptr_->wait_for_service()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Service server not available after waiting");
    rclcpp::shutdown();
  }

  auto request = std::make_shared<Msg::Request>();
  request->order = 10;

  RCLCPP_INFO(this->get_logger(), "Sending request");
#if 0
  auto result = client_ptr_->async_send_request(request);
  // TODO: Segmentation faultの発生を確認
  // Attention: rclcpp::spinとの多重spin発生に気を付ける
  auto return_code =
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), result);

  if (return_code == rclcpp::FutureReturnCode::SUCCESS) {
    std::stringstream ss;
    ss << "Result received: ";
    for (auto number : result.get()->sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Failed to call service add_two_ints");
  }
#else
  auto result = client_ptr_->async_send_request(
    request, [this](rclcpp::Client<Msg>::SharedFuture future) {
      std::stringstream ss;
      ss << "Result received: ";
      for (auto number : future.get()->sequence) {
        ss << number << " ";
      }
      RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    });
#endif
};

} // namespace fibonacci

// #include "rclcpp_components/register_node_macro.hpp"
// RCLCPP_COMPONENTS_REGISTER_NODE(fibonacci::FibonacciServiceClient)
