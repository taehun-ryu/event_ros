#ifndef EVENT_BRIDGE_CPP__VISIBILITY_CONTROL_H_
#define EVENT_BRIDGE_CPP__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define EVENT_BRIDGE_CPP_EXPORT __attribute__ ((dllexport))
    #define EVENT_BRIDGE_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define EVENT_BRIDGE_CPP_EXPORT __declspec(dllexport)
    #define EVENT_BRIDGE_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef EVENT_BRIDGE_CPP_BUILDING_LIBRARY
    #define EVENT_BRIDGE_CPP_PUBLIC EVENT_BRIDGE_CPP_EXPORT
  #else
    #define EVENT_BRIDGE_CPP_PUBLIC EVENT_BRIDGE_CPP_IMPORT
  #endif
  #define EVENT_BRIDGE_CPP_PUBLIC_TYPE EVENT_BRIDGE_CPP_PUBLIC
  #define EVENT_BRIDGE_CPP_LOCAL
#else
  #define EVENT_BRIDGE_CPP_EXPORT __attribute__ ((visibility("default")))
  #define EVENT_BRIDGE_CPP_IMPORT
  #if __GNUC__ >= 4
    #define EVENT_BRIDGE_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define EVENT_BRIDGE_CPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define EVENT_BRIDGE_CPP_PUBLIC
    #define EVENT_BRIDGE_CPP_LOCAL
  #endif
  #define EVENT_BRIDGE_CPP_PUBLIC_TYPE
#endif

#endif  // EVENT_BRIDGE_CPP__VISIBILITY_CONTROL_H_
