#include "fibonacci/service_server.hpp"
// STL
#include <functional>

namespace fibonacci {
/// @brief コンストラクタ
/// @param options ROS2のノード設定
ServiceServer::ServiceServer(const rclcpp::NodeOptions& options)
  : Node(node_name, options)
{
  using namespace std::placeholders;
  RCLCPP_DEBUG(get_logger(), "Establish Server");
  server = create_service<Msg>(
    service_name, std::bind(&ServiceServer::execute, this, _1, _2));
  RCLCPP_INFO_STREAM(get_logger(), get_name() << " created");
}

/// @brief デストラクタ
ServiceServer::~ServiceServer()
{
  RCLCPP_INFO_STREAM(get_logger(), get_name() << " finalized");
}

/// @brief サービス実行関数
/// @param request リクエスト情報
/// @param response レスポンス情報
void
ServiceServer::execute(const std::shared_ptr<Msg::Request> request,
                       std::shared_ptr<Msg::Response> response)
{
  RCLCPP_INFO(get_logger(), "Executing fibonacci Service");

  auto& sequence = response->sequence;
  sequence.push_back(0);
  sequence.push_back(1);

  for (int i = 1; (i < request->order) && rclcpp::ok(); ++i) {
    // Update sequence
    sequence.push_back(sequence[i] + sequence[i - 1]);
  }

  RCLCPP_INFO(get_logger(), "Request was succeeded");
}
} // namespace fibonacci

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fibonacci::ServiceServer)
