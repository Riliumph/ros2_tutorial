// STL
#include <iostream>
#include <memory>
#include <sstream>
// ROS2
#include <rclcpp/rclcpp.hpp>
// original
#include "fibonacci/action_client.hpp"

const char* node_name = "rclcpp";

int
main(int argc, char** argv)
{
  auto logger = rclcpp::get_logger(node_name);
  if (argc < 2) {
    RCLCPP_ERROR(logger, "Usage: node <arg1>");
    return 1;
  }
  rclcpp::init(argc, argv);
  pid_t pid = getpid();
  RCLCPP_INFO(logger, "プロセスID: %d", pid);
  auto cli = std::make_shared<fibonacci::ActionClient>();
  auto goal = fibonacci::ActionClient::Msg::Goal();

  try {
    std::string arg1(argv[1]);
    goal.order = std::stoi(arg1);
    auto response = cli->Send(goal);
    if (!response) {
      RCLCPP_INFO(logger, "failed to send");
      rclcpp::shutdown();
      return -1;
    }
    switch (response->code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(logger, "Request was succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_INFO(logger, "Request was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(logger, "Request was canceled");
        break;
      default:
        RCLCPP_ERROR(logger,
                     "unexpected return code(%d)",
                     static_cast<int8_t>(response->code));
        break;
    }
    if (response->result) {
      RCLCPP_INFO_STREAM(logger, "Received: " << *(response->result));
    }
  } catch (const std::exception& e) {
    RCLCPP_INFO(logger, e.what());
  }

  rclcpp::shutdown();
  return 0;
}
