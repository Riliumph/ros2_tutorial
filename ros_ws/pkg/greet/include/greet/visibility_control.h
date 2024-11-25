#ifndef GREET__VISIBILITY_CONTROL_H_
#define GREET__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define GREET_EXPORT __attribute__((dllexport))
#define GREET_IMPORT __attribute__((dllimport))
#else
#define GREET_EXPORT __declspec(dllexport)
#define GREET_IMPORT __declspec(dllimport)
#endif
#ifdef GREET_BUILDING_LIBRARY
#define GREET_PUBLIC GREET_EXPORT
#else
#define GREET_PUBLIC GREET_IMPORT
#endif
#define GREET_PUBLIC_TYPE GREET_PUBLIC
#define GREET_LOCAL
#else
#define GREET_EXPORT __attribute__((visibility("default")))
#define GREET_IMPORT
#if __GNUC__ >= 4
#define GREET_PUBLIC __attribute__((visibility("default")))
#define GREET_LOCAL __attribute__((visibility("hidden")))
#else
#define GREET_PUBLIC
#define GREET_LOCAL
#endif
#define GREET_PUBLIC_TYPE
#endif

#endif // GREET__VISIBILITY_CONTROL_H_
