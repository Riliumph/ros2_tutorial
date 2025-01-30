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
  if (argc < 2) {
    RCLCPP_ERROR(rclcpp::get_logger(node_name), "Usage: node <arg1>");
    return 1;
  }
  rclcpp::init(argc, argv);
  pid_t pid = getpid();
  RCLCPP_INFO(rclcpp::get_logger(node_name), "プロセスID: %d", pid);
  auto cli = std::make_shared<fibonacci::ActionClient>();
  auto goal = fibonacci::ActionClient::Msg::Goal();

  try {
    std::string arg1(argv[1]);
    goal.order = std::stoi(arg1);
    auto response = cli->Send(goal);
    if (!response) {
      RCLCPP_INFO(rclcpp::get_logger(node_name), "failed to send");
      rclcpp::shutdown();
      return -1;
    }
    switch (response->code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(rclcpp::get_logger(node_name), "Request was succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_INFO(rclcpp::get_logger(node_name), "Request was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(rclcpp::get_logger(node_name), "Request was canceled");
        break;
      default:
        RCLCPP_ERROR(rclcpp::get_logger(node_name),
                     "unexpected return code(%d)",
                     static_cast<int8_t>(response->code));
        break;
    }
    if (response->result) {
      std::cout << "Received: " << *(response->result) << std::endl;
    }
  } catch (const std::exception& e) {
    RCLCPP_INFO(rclcpp::get_logger(node_name), e.what());
  }

  rclcpp::shutdown();
  return 0;
}
