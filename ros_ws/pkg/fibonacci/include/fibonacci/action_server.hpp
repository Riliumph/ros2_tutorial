#ifndef FIBONACCI__ACTION_SERVER_HPP_
#define FIBONACCI__ACTION_SERVER_HPP_
// STL
#include <memory>
// ROS2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
// ROS2 automatically created definition
#include "fibonacci/visibility_control.h"
#include "fibonacci_msg/action/operator_io.hpp"

namespace fibonacci {
/// @brief N回までのFibonacci数を算出して返すサービス
/// Resultで指定された回数場のFibonacci数列を返却する
/// Feedbackとして一回計算がされるごとに途中経過も
class ActionServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  using Msg = fibonacci_msg::action::Fibonacci;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Msg>;

public:
  static constexpr const char* node_name = "fibonacci_action_server_node";
  static constexpr const char* service_name = "fibonacci_action_service";
  static void graceful_shutdown(int);

public: // action server
  FIBONACCI_PUBLIC explicit ActionServer(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~ActionServer();

public: // lifecycle node override
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State&) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State&) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State&) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State&) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State&) override;

private:
  rclcpp_action::Server<Msg>::SharedPtr server;
  void execute(std::shared_ptr<GoalHandle>);

private: // callback
  rclcpp_action::GoalResponse Receive(const rclcpp_action::GoalUUID&,
                                      std::shared_ptr<const Msg::Goal>);
  rclcpp_action::CancelResponse Cancel(std::shared_ptr<GoalHandle>);
  void Accept(std::shared_ptr<GoalHandle>);
};

} // namespace fibonacci

#endif // FIBONACCI__ACTION_SERVER_HPP_
