#ifndef EDUART_ROBOT__VISIBILITY_CONTROL_H_
#define EDUART_ROBOT__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define EDUART_ROBOT_EXPORT __attribute__ ((dllexport))
    #define EDUART_ROBOT_IMPORT __attribute__ ((dllimport))
  #else
    #define EDUART_ROBOT_EXPORT __declspec(dllexport)
    #define EDUART_ROBOT_IMPORT __declspec(dllimport)
  #endif
  #ifdef EDUART_ROBOT_BUILDING_LIBRARY
    #define EDUART_ROBOT_PUBLIC EDUART_ROBOT_EXPORT
  #else
    #define EDUART_ROBOT_PUBLIC EDUART_ROBOT_IMPORT
  #endif
  #define EDUART_ROBOT_PUBLIC_TYPE EDUART_ROBOT_PUBLIC
  #define EDUART_ROBOT_LOCAL
#else
  #define EDUART_ROBOT_EXPORT __attribute__ ((visibility("default")))
  #define EDUART_ROBOT_IMPORT
  #if __GNUC__ >= 4
    #define EDUART_ROBOT_PUBLIC __attribute__ ((visibility("default")))
    #define EDUART_ROBOT_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define EDUART_ROBOT_PUBLIC
    #define EDUART_ROBOT_LOCAL
  #endif
  #define EDUART_ROBOT_PUBLIC_TYPE
#endif

#endif  // EDUART_ROBOT__VISIBILITY_CONTROL_H_
