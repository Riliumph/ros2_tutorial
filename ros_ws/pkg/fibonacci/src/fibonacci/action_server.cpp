#include "fibonacci/action_server.hpp"
// STL
#include <functional>
#include <thread>
// original
#include "fibonacci/action_operator_io.hpp"

namespace fibonacci {
/// @brief コンストラクタ
/// @param options ROS2のノード設定
ActionServer::ActionServer(const rclcpp::NodeOptions& options)
  : Node(node_name, options)
{
  using namespace std::placeholders;
  RCLCPP_DEBUG(get_logger(), "Establish Server");
  server = rclcpp_action::create_server<Msg>(
    this,
    service_name,
    std::bind(&ActionServer::Receive, this, _1, _2),
    std::bind(&ActionServer::Cancel, this, _1),
    std::bind(&ActionServer::Accept, this, _1));
  RCLCPP_INFO_STREAM(get_logger(), get_name() << " created");
}

/// @brief デストラクタ
ActionServer::~ActionServer()
{
  RCLCPP_INFO_STREAM(get_logger(), get_name() << " finalized");
}

/// @brief サーバーの業務処理
/// @param request リクエスト情報
void
ActionServer::execute(const std::shared_ptr<GoalHandle> request)
{
  RCLCPP_INFO(get_logger(), "=== NEW REQUEST RECEIVED ===");
  RCLCPP_INFO(get_logger(), "Executing fibonacci service");
  rclcpp::Rate loop_rate(1);
  const auto goal = request->get_goal();
  auto feedback = std::make_shared<Msg::Feedback>();
  auto& sequence = feedback->partial_sequence;
  sequence.push_back(0);
  sequence.push_back(1);
  auto result = std::make_shared<Msg::Result>();

  for (int i = 1; i < goal->order; ++i) {
    if (request->is_canceling()) {
      break;
    }
    // Update sequence
    sequence.push_back(sequence[i] + sequence[i - 1]);
    // Publish feedback
    request->publish_feedback(feedback);
    RCLCPP_INFO_STREAM(get_logger(), "Publish feedback: " << *feedback);
    loop_rate.sleep();
  }

  result->sequence = sequence;
  if (request->is_canceling()) {
    request->canceled(result);
    RCLCPP_INFO(get_logger(), "Request was canceled");
  } else {
    request->succeed(result);
    RCLCPP_INFO(get_logger(), "Request was succeeded");
  }
}

/// @brief Goalリクエスト受信時に発火するコールバック
/// @param uuid リクエストIDに相当するID
/// @param request クライアントから受信したデータ
/// @return 実行許可状態
rclcpp_action::GoalResponse
ActionServer::Receive(const rclcpp_action::GoalUUID& uuid,
                      std::shared_ptr<const Msg::Goal> request)
{
  RCLCPP_INFO(get_logger(), "Receive goal request");
  RCLCPP_INFO_STREAM(get_logger(), "Request ID: " << uuid);

  if (request->order < 0) {
    RCLCPP_INFO(get_logger(), "Request was rejected");
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(get_logger(), "Request was accepted");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/// @brief キャンセルリクエスト受信時に発火するコールバック
/// @param request クライアントから受信したデータ
/// @return キャンセル許可
rclcpp_action::CancelResponse
ActionServer::Cancel(std::shared_ptr<GoalHandle> request)
{
  RCLCPP_INFO(get_logger(), "Receive cancel request");
  (void)request;
  return rclcpp_action::CancelResponse::ACCEPT;
}

/// @brief 受信リクエスト受信時に発火するコールバック
/// @param request クライアントから受信したデータ
void
ActionServer::Accept(std::shared_ptr<GoalHandle> request)
{
  using namespace std::placeholders;
  RCLCPP_INFO(get_logger(), "Start execution of goal");
  // this needs to return quickly to avoid blocking the executor,
  // so spin up a new thread
  std::thread{ std::bind(&ActionServer::execute, this, _1), request }.detach();
}
} // namespace fibonacci

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fibonacci::ActionServer)
