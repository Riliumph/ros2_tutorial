#include "graceful_shutdown/flags.hpp"
std::atomic<bool> graceful_shutdown(false);
std::atomic<bool> keep_check_graceful_shutdown(false);
