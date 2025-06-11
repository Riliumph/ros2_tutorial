// STL
#include <csignal>
#include <iostream>
#include <memory>
#include <sstream>
#include <unistd.h>
// ROS2
#include <rclcpp/rclcpp.hpp>
// original
#include "fibonacci/action_client.hpp"
#include "graceful_shutdown/flags.hpp"

const char* node_name = "your_rclcpp";

void
GracefulShutdown(int sig)
{
  RCLCPP_INFO(rclcpp::get_logger(node_name), "Caught signal %d", sig);
  graceful_shutdown.store(true);
}

void
CheckGracefulShutdown(std::shared_ptr<fibonacci::ActionClient> cli)
{
  RCLCPP_INFO(rclcpp::get_logger(node_name), "start to check gs");
  keep_check_graceful_shutdown.store(true, std::memory_order_relaxed);
  while (keep_check_graceful_shutdown.load(std::memory_order_relaxed)) {
    if (graceful_shutdown.load(std::memory_order_relaxed)) {
      RCLCPP_INFO(rclcpp::get_logger(node_name), "graceful shutdown required");
      cli->Abort();
      RCLCPP_INFO(rclcpp::get_logger(node_name), "cancel completed");
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(rclcpp::get_logger(node_name), "thread terminated");
}

int
main(int argc, char** argv)
{
  auto logger = rclcpp::get_logger(node_name);
  if (argc < 2) {
    RCLCPP_ERROR(logger, "Usage: node <arg1>");
    return 1;
  }

  struct sigaction sa;
  sa.sa_handler = GracefulShutdown;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  sigaction(SIGINT, &sa, nullptr);

  rclcpp::init(
    argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  pid_t pid = getpid();
  RCLCPP_INFO(logger, "プロセスID: %d", pid);
  auto cli = std::make_shared<fibonacci::ActionClient>();
  auto goal = fibonacci::ActionClient::Msg::Goal();

  std::thread gs_checker(CheckGracefulShutdown, cli);

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
  keep_check_graceful_shutdown.store(false, std::memory_order_relaxed);
  RCLCPP_INFO(logger, "joining gs_checker");
  gs_checker.join();
  rclcpp::shutdown();
  return 0;
}
