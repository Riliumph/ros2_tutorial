#ifndef FIBONACCI_MSG__ACTION_OPERATOR_IO_HPP_
#define FIBONACCI_MSG__ACTION_OPERATOR_IO_HPP_
#include "fibonacci_msg/action/fibonacci.hpp"
// STL
#include <ostream>

std::ostream&
operator<<(std::ostream& os, const fibonacci_msg::action::Fibonacci::Goal& lv)
{
  os << lv.order;
  return os;
}

std::ostream&
operator<<(std::ostream& os, const fibonacci_msg::action::Fibonacci::Result& lv)
{
  for (const auto& s : lv.sequence) {
    os << s << " ";
  }
  return os;
}

std::ostream&
operator<<(std::ostream& os,
           const fibonacci_msg::action::Fibonacci::Feedback& lv)
{
  for (const auto& s : lv.partial_sequence) {
    os << s << " ";
  }
  return os;
}

#endif // FIBONACCI_MSG__ACTION_OPERATOR_IO_HPP_
