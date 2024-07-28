// STL
#include <memory>
// ROS
#include <rclcpp/rclcpp.hpp>

#include <fibonacci/action_server.hpp>
#include <fibonacci/service_server.hpp>

int
main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  // タイマーコールバック、トピックコールバック等を行うexecutor
  rclcpp::executors::SingleThreadedExecutor exec;

  auto as = std::make_shared<fibonacci::FibonacciActionServer>(options);
  exec.add_node(as);
  auto ss = std::make_shared<fibonacci::FibonacciServiceServer>(options);
  exec.add_node(ss);

  // Ctrl-Cが押されるまでexecutorを実行する
  exec.spin();
  // Shut down ROS
  rclcpp::shutdown();
  return 0;
}
