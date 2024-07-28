#include "fibonacci/service_server.hpp"

namespace fibonacci {

/// @brief コンストラクタ
/// @param options ROS2のノード設定
FibonacciServiceServer::FibonacciServiceServer(
  const rclcpp::NodeOptions& options)
  : Node(node_name, options)
{
  RCLCPP_DEBUG(this->get_logger(), "Establish Server");
  service_server_ =
    this->create_service<Msg>(server_name,
                              std::bind(&FibonacciServiceServer::execute,
                                        this,
                                        std::placeholders::_1,
                                        std::placeholders::_2));
};

/// @brief サービス実行関数
/// @param request リクエスト情報
/// @param response レスポンス情報
void
FibonacciServiceServer::execute(const std::shared_ptr<Msg::Request> request,
                                std::shared_ptr<Msg::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Executing Fibonacci Service");

  auto& sequence = response->sequence;
  sequence.push_back(0);
  sequence.push_back(1);

  for (int i = 1; (i < request->order) && rclcpp::ok(); ++i) {
    // Update sequence
    sequence.push_back(sequence[i] + sequence[i - 1]);
  }

  RCLCPP_INFO(this->get_logger(), "Fibonacci Service succeeded");
};

} // namespace fibonacci

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fibonacci::FibonacciServiceServer)
