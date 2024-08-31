#ifndef FIBONACCI_MSG__SRV_OPERATOR_IO_HPP_
#define FIBONACCI_MSG__SRV_OPERATOR_IO_HPP_
#include "fibonacci_msg/srv/fibonacci.hpp"
// STL
#include <ostream>

std::ostream&
operator<<(std::ostream& os, const fibonacci_msg::srv::Fibonacci::Request& lv)
{
  os << lv.order;
  return os;
}

std::ostream&
operator<<(std::ostream& os, const fibonacci_msg::srv::Fibonacci::Response& lv)
{
  for (const auto& s : lv.sequence) {
    os << s << " ";
  }
  return os;
}

#endif // FIBONACCI_MSG__SRV_OPERATOR_IO_HPP_
