#include "fibonacci/action_server.hpp"

namespace fibonacci {

/// @brief コンストラクタ
/// @param options ROS2のノード設定
FibonacciActionServer::FibonacciActionServer(const rclcpp::NodeOptions& options)
  : Node(node_name, options)
{
  RCLCPP_DEBUG(this->get_logger(), "Establish Server");
  auto goal_handler = [this](const rclcpp_action::GoalUUID& uuid,
                             std::shared_ptr<const ActMsg::Goal> goal) {
    RCLCPP_INFO(
      this->get_logger(), "Received goal request with order %d", goal->order);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  };

  auto cancel_handler = [this](const std::shared_ptr<GoalHandle> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  };

  auto handle_accepted = [this](const std::shared_ptr<GoalHandle> goal_handle) {
    // エグゼキュータをブロックしないように迅速に返す必要があるので、
    // 新しいスレッド内で呼び出されるラムダ関数を宣言します
    auto execute_in_thread = [this, goal_handle]() {
      return this->execute(goal_handle);
    };
    std::thread{ execute_in_thread }.detach();
  };

  action_server_ = rclcpp_action::create_server<ActMsg>(
    this, server_name, goal_handler, cancel_handler, handle_accepted);
}

/// @brief 実行関数
/// @param goal_handle
void
FibonacciActionServer::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  rclcpp::Rate loop_rate(1);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<ActMsg::Feedback>();
  auto& sequence = feedback->partial_sequence;
  sequence.push_back(0);
  sequence.push_back(1);
  auto result = std::make_shared<ActMsg::Result>();

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

} // namespace fibonacci

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fibonacci::FibonacciActionServer)
