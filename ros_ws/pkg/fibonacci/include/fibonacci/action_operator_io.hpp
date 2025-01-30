#ifndef FIBONACCI__ACTION_OPERATOR_IO_HPP_
#define FIBONACCI__ACTION_OPERATOR_IO_HPP_
#include <rclcpp_action/rclcpp_action.hpp>
// STL
#include <ostream>
#include <sstream>

std::ostream&
operator<<(std::ostream& os, const rclcpp_action::GoalUUID& uuid)
{
  std::stringstream ss;
  ss << std::hex << std::setfill('0') << std::setw(2);
  for (const auto& id : uuid) {
    ss << static_cast<int>(id);
  }
  os << ss.str();
  return os;
}

std::ostream&
operator<<(std::ostream& os, const rclcpp_action::ResultCode& code)
{
  switch (code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      os << "SUCCEEDED";
      break;
    case rclcpp_action::ResultCode::ABORTED:
      os << "ABORTED";
      break;
    case rclcpp_action::ResultCode::CANCELED:
      os << "CANCELED";
      break;
    default:
      os << "UNKNOWN";
      break;
  }
  return os;
}

#endif // FIBONACCI__ACTION_OPERATOR_IO_HPP_
