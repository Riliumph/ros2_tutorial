#ifndef FIBONACCI__VISIBILITY_CONTROL_H_
#define FIBONACCI__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define FIBONACCI_EXPORT __attribute__ ((dllexport))
    #define FIBONACCI_IMPORT __attribute__ ((dllimport))
  #else
    #define FIBONACCI_EXPORT __declspec(dllexport)
    #define FIBONACCI_IMPORT __declspec(dllimport)
  #endif
  #ifdef FIBONACCI_BUILDING_LIBRARY
    #define FIBONACCI_PUBLIC FIBONACCI_EXPORT
  #else
    #define FIBONACCI_PUBLIC FIBONACCI_IMPORT
  #endif
  #define FIBONACCI_PUBLIC_TYPE FIBONACCI_PUBLIC
  #define FIBONACCI_LOCAL
#else
  #define FIBONACCI_EXPORT __attribute__ ((visibility("default")))
  #define FIBONACCI_IMPORT
  #if __GNUC__ >= 4
    #define FIBONACCI_PUBLIC __attribute__ ((visibility("default")))
    #define FIBONACCI_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define FIBONACCI_PUBLIC
    #define FIBONACCI_LOCAL
  #endif
  #define FIBONACCI_PUBLIC_TYPE
#endif

#endif  // FIBONACCI__VISIBILITY_CONTROL_H_
