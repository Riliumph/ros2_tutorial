#ifndef INCLUDE_DEFINE_HPP
#define INCLUDE_DEFINE_HPP
#include <atomic>
extern std::atomic<bool> graceful_shutdown;
extern std::atomic<bool> keep_check_graceful_shutdown;
#endif // INCLUDE_DEFINE_HPP
