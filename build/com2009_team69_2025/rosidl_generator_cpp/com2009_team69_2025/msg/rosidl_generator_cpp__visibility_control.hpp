// generated from rosidl_generator_cpp/resource/rosidl_generator_cpp__visibility_control.hpp.in
// generated code does not contain a copyright notice

#ifndef COM2009_TEAM69_2025__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_
#define COM2009_TEAM69_2025__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_GENERATOR_CPP_EXPORT_com2009_team69_2025 __attribute__ ((dllexport))
    #define ROSIDL_GENERATOR_CPP_IMPORT_com2009_team69_2025 __attribute__ ((dllimport))
  #else
    #define ROSIDL_GENERATOR_CPP_EXPORT_com2009_team69_2025 __declspec(dllexport)
    #define ROSIDL_GENERATOR_CPP_IMPORT_com2009_team69_2025 __declspec(dllimport)
  #endif
  #ifdef ROSIDL_GENERATOR_CPP_BUILDING_DLL_com2009_team69_2025
    #define ROSIDL_GENERATOR_CPP_PUBLIC_com2009_team69_2025 ROSIDL_GENERATOR_CPP_EXPORT_com2009_team69_2025
  #else
    #define ROSIDL_GENERATOR_CPP_PUBLIC_com2009_team69_2025 ROSIDL_GENERATOR_CPP_IMPORT_com2009_team69_2025
  #endif
#else
  #define ROSIDL_GENERATOR_CPP_EXPORT_com2009_team69_2025 __attribute__ ((visibility("default")))
  #define ROSIDL_GENERATOR_CPP_IMPORT_com2009_team69_2025
  #if __GNUC__ >= 4
    #define ROSIDL_GENERATOR_CPP_PUBLIC_com2009_team69_2025 __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_GENERATOR_CPP_PUBLIC_com2009_team69_2025
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // COM2009_TEAM69_2025__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_
