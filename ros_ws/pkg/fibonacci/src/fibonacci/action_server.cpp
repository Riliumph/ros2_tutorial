#include "fibonacci/action_server.hpp"
// STL
#include <functional>
#include <thread>

namespace fibonacci {

/// @brief コンストラクタ
/// @param options ROS2のノード設定
FibonacciActionServer::FibonacciActionServer(const rclcpp::NodeOptions& options)
  : Node(node_name, options)
{
  using namespace std::placeholders;
  RCLCPP_DEBUG(this->get_logger(), "Establish Server");
  server = rclcpp_action::create_server<Msg>(
    this,
    server_name,
    std::bind(&FibonacciActionServer::Receive, this, _1, _2),
    std::bind(&FibonacciActionServer::Cancel, this, _1),
    std::bind(&FibonacciActionServer::Accept, this, _1));
}

/// @brief 実行関数
/// @param goal_handle
void
FibonacciActionServer::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  rclcpp::Rate loop_rate(1);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Msg::Feedback>();
  auto& sequence = feedback->partial_sequence;
  sequence.push_back(0);
  sequence.push_back(1);
  auto result = std::make_shared<Msg::Result>();

  for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
    // Check if there is a cancel request
    if (goal_handle->is_canceling()) {
      result->sequence = sequence;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }
    // Update sequence
    sequence.push_back(sequence[i] + sequence[i - 1]);
    // Publish feedback
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "Publish feedback");

    loop_rate.sleep();
  }

  // Check if goal is done
  if (rclcpp::ok()) {
    result->sequence = sequence;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
};

/// @brief Goalリクエスト受信時に発火するコールバック
/// @param uuid リクエストIDに相当するID
/// @param request クライアントから受信したデータ
/// @return 実行許可
rclcpp_action::GoalResponse
FibonacciActionServer::Receive(const rclcpp_action::GoalUUID& uuid,
                               std::shared_ptr<const Msg::Goal> request)
{
  RCLCPP_INFO(this->get_logger(), "Receive goal request");
  std::stringstream ss;
  ss << std::hex << std::setfill('0') << std::setw(2);
  for (const auto& id : uuid) {
    ss << static_cast<int>(id);
  }
  RCLCPP_INFO(this->get_logger(), "Request ID: %s", ss.str().data());
  if (request->order < 0) {
    RCLCPP_INFO(this->get_logger(), "Request was rejected");
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(this->get_logger(), "Request was accepted");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/// @brief キャンセルリクエスト受信時に発火するコールバック
/// @param request クライアントから受信したデータ
/// @return キャンセル許可
rclcpp_action::CancelResponse
FibonacciActionServer::Cancel(std::shared_ptr<GoalHandle> request)
{
  RCLCPP_INFO(this->get_logger(), "Receive cancel request");
  (void)request;
  return rclcpp_action::CancelResponse::ACCEPT;
}

/// @brief 受信リクエスト受信時に発火するコールバック
/// @param request クライアントから受信したデータ
void
FibonacciActionServer::Accept(std::shared_ptr<GoalHandle> request)
{
  using namespace std::placeholders;
  RCLCPP_INFO(this->get_logger(), "Beginning execution of goal");
  // this needs to return quickly to avoid blocking the executor,
  // so spin up a new thread
  std::thread{ std::bind(&FibonacciActionServer::execute, this, _1), request }
    .detach();
}

} // namespace fibonacci

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fibonacci::FibonacciActionServer)
