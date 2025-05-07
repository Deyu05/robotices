// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from com2009_team69_2025:action/ExploreForward.idl
// generated code does not contain a copyright notice

#ifndef COM2009_TEAM69_2025__ACTION__DETAIL__EXPLORE_FORWARD__TRAITS_HPP_
#define COM2009_TEAM69_2025__ACTION__DETAIL__EXPLORE_FORWARD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "com2009_team69_2025/action/detail/explore_forward__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace com2009_team69_2025
{

namespace action
{

inline void to_flow_style_yaml(
  const ExploreForward_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: fwd_velocity
  {
    out << "fwd_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.fwd_velocity, out);
    out << ", ";
  }

  // member: stopping_distance
  {
    out << "stopping_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.stopping_distance, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExploreForward_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: fwd_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fwd_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.fwd_velocity, out);
    out << "\n";
  }

  // member: stopping_distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stopping_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.stopping_distance, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExploreForward_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace com2009_team69_2025

namespace rosidl_generator_traits
{

[[deprecated("use com2009_team69_2025::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const com2009_team69_2025::action::ExploreForward_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  com2009_team69_2025::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use com2009_team69_2025::action::to_yaml() instead")]]
inline std::string to_yaml(const com2009_team69_2025::action::ExploreForward_Goal & msg)
{
  return com2009_team69_2025::action::to_yaml(msg);
}

template<>
inline const char * data_type<com2009_team69_2025::action::ExploreForward_Goal>()
{
  return "com2009_team69_2025::action::ExploreForward_Goal";
}

template<>
inline const char * name<com2009_team69_2025::action::ExploreForward_Goal>()
{
  return "com2009_team69_2025/action/ExploreForward_Goal";
}

template<>
struct has_fixed_size<com2009_team69_2025::action::ExploreForward_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<com2009_team69_2025::action::ExploreForward_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<com2009_team69_2025::action::ExploreForward_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace com2009_team69_2025
{

namespace action
{

inline void to_flow_style_yaml(
  const ExploreForward_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: total_distance_travelled
  {
    out << "total_distance_travelled: ";
    rosidl_generator_traits::value_to_yaml(msg.total_distance_travelled, out);
    out << ", ";
  }

  // member: closest_obstacle
  {
    out << "closest_obstacle: ";
    rosidl_generator_traits::value_to_yaml(msg.closest_obstacle, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExploreForward_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: total_distance_travelled
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total_distance_travelled: ";
    rosidl_generator_traits::value_to_yaml(msg.total_distance_travelled, out);
    out << "\n";
  }

  // member: closest_obstacle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "closest_obstacle: ";
    rosidl_generator_traits::value_to_yaml(msg.closest_obstacle, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExploreForward_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace com2009_team69_2025

namespace rosidl_generator_traits
{

[[deprecated("use com2009_team69_2025::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const com2009_team69_2025::action::ExploreForward_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  com2009_team69_2025::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use com2009_team69_2025::action::to_yaml() instead")]]
inline std::string to_yaml(const com2009_team69_2025::action::ExploreForward_Result & msg)
{
  return com2009_team69_2025::action::to_yaml(msg);
}

template<>
inline const char * data_type<com2009_team69_2025::action::ExploreForward_Result>()
{
  return "com2009_team69_2025::action::ExploreForward_Result";
}

template<>
inline const char * name<com2009_team69_2025::action::ExploreForward_Result>()
{
  return "com2009_team69_2025/action/ExploreForward_Result";
}

template<>
struct has_fixed_size<com2009_team69_2025::action::ExploreForward_Result>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<com2009_team69_2025::action::ExploreForward_Result>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<com2009_team69_2025::action::ExploreForward_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace com2009_team69_2025
{

namespace action
{

inline void to_flow_style_yaml(
  const ExploreForward_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: current_distance_travelled
  {
    out << "current_distance_travelled: ";
    rosidl_generator_traits::value_to_yaml(msg.current_distance_travelled, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExploreForward_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: current_distance_travelled
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_distance_travelled: ";
    rosidl_generator_traits::value_to_yaml(msg.current_distance_travelled, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExploreForward_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace com2009_team69_2025

namespace rosidl_generator_traits
{

[[deprecated("use com2009_team69_2025::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const com2009_team69_2025::action::ExploreForward_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  com2009_team69_2025::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use com2009_team69_2025::action::to_yaml() instead")]]
inline std::string to_yaml(const com2009_team69_2025::action::ExploreForward_Feedback & msg)
{
  return com2009_team69_2025::action::to_yaml(msg);
}

template<>
inline const char * data_type<com2009_team69_2025::action::ExploreForward_Feedback>()
{
  return "com2009_team69_2025::action::ExploreForward_Feedback";
}

template<>
inline const char * name<com2009_team69_2025::action::ExploreForward_Feedback>()
{
  return "com2009_team69_2025/action/ExploreForward_Feedback";
}

template<>
struct has_fixed_size<com2009_team69_2025::action::ExploreForward_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<com2009_team69_2025::action::ExploreForward_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<com2009_team69_2025::action::ExploreForward_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "com2009_team69_2025/action/detail/explore_forward__traits.hpp"

namespace com2009_team69_2025
{

namespace action
{

inline void to_flow_style_yaml(
  const ExploreForward_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExploreForward_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExploreForward_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace com2009_team69_2025

namespace rosidl_generator_traits
{

[[deprecated("use com2009_team69_2025::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const com2009_team69_2025::action::ExploreForward_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  com2009_team69_2025::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use com2009_team69_2025::action::to_yaml() instead")]]
inline std::string to_yaml(const com2009_team69_2025::action::ExploreForward_SendGoal_Request & msg)
{
  return com2009_team69_2025::action::to_yaml(msg);
}

template<>
inline const char * data_type<com2009_team69_2025::action::ExploreForward_SendGoal_Request>()
{
  return "com2009_team69_2025::action::ExploreForward_SendGoal_Request";
}

template<>
inline const char * name<com2009_team69_2025::action::ExploreForward_SendGoal_Request>()
{
  return "com2009_team69_2025/action/ExploreForward_SendGoal_Request";
}

template<>
struct has_fixed_size<com2009_team69_2025::action::ExploreForward_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<com2009_team69_2025::action::ExploreForward_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<com2009_team69_2025::action::ExploreForward_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<com2009_team69_2025::action::ExploreForward_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<com2009_team69_2025::action::ExploreForward_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace com2009_team69_2025
{

namespace action
{

inline void to_flow_style_yaml(
  const ExploreForward_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExploreForward_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExploreForward_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace com2009_team69_2025

namespace rosidl_generator_traits
{

[[deprecated("use com2009_team69_2025::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const com2009_team69_2025::action::ExploreForward_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  com2009_team69_2025::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use com2009_team69_2025::action::to_yaml() instead")]]
inline std::string to_yaml(const com2009_team69_2025::action::ExploreForward_SendGoal_Response & msg)
{
  return com2009_team69_2025::action::to_yaml(msg);
}

template<>
inline const char * data_type<com2009_team69_2025::action::ExploreForward_SendGoal_Response>()
{
  return "com2009_team69_2025::action::ExploreForward_SendGoal_Response";
}

template<>
inline const char * name<com2009_team69_2025::action::ExploreForward_SendGoal_Response>()
{
  return "com2009_team69_2025/action/ExploreForward_SendGoal_Response";
}

template<>
struct has_fixed_size<com2009_team69_2025::action::ExploreForward_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<com2009_team69_2025::action::ExploreForward_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<com2009_team69_2025::action::ExploreForward_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<com2009_team69_2025::action::ExploreForward_SendGoal>()
{
  return "com2009_team69_2025::action::ExploreForward_SendGoal";
}

template<>
inline const char * name<com2009_team69_2025::action::ExploreForward_SendGoal>()
{
  return "com2009_team69_2025/action/ExploreForward_SendGoal";
}

template<>
struct has_fixed_size<com2009_team69_2025::action::ExploreForward_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<com2009_team69_2025::action::ExploreForward_SendGoal_Request>::value &&
    has_fixed_size<com2009_team69_2025::action::ExploreForward_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<com2009_team69_2025::action::ExploreForward_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<com2009_team69_2025::action::ExploreForward_SendGoal_Request>::value &&
    has_bounded_size<com2009_team69_2025::action::ExploreForward_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<com2009_team69_2025::action::ExploreForward_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<com2009_team69_2025::action::ExploreForward_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<com2009_team69_2025::action::ExploreForward_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace com2009_team69_2025
{

namespace action
{

inline void to_flow_style_yaml(
  const ExploreForward_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExploreForward_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExploreForward_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace com2009_team69_2025

namespace rosidl_generator_traits
{

[[deprecated("use com2009_team69_2025::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const com2009_team69_2025::action::ExploreForward_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  com2009_team69_2025::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use com2009_team69_2025::action::to_yaml() instead")]]
inline std::string to_yaml(const com2009_team69_2025::action::ExploreForward_GetResult_Request & msg)
{
  return com2009_team69_2025::action::to_yaml(msg);
}

template<>
inline const char * data_type<com2009_team69_2025::action::ExploreForward_GetResult_Request>()
{
  return "com2009_team69_2025::action::ExploreForward_GetResult_Request";
}

template<>
inline const char * name<com2009_team69_2025::action::ExploreForward_GetResult_Request>()
{
  return "com2009_team69_2025/action/ExploreForward_GetResult_Request";
}

template<>
struct has_fixed_size<com2009_team69_2025::action::ExploreForward_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<com2009_team69_2025::action::ExploreForward_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<com2009_team69_2025::action::ExploreForward_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "com2009_team69_2025/action/detail/explore_forward__traits.hpp"

namespace com2009_team69_2025
{

namespace action
{

inline void to_flow_style_yaml(
  const ExploreForward_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExploreForward_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExploreForward_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace com2009_team69_2025

namespace rosidl_generator_traits
{

[[deprecated("use com2009_team69_2025::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const com2009_team69_2025::action::ExploreForward_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  com2009_team69_2025::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use com2009_team69_2025::action::to_yaml() instead")]]
inline std::string to_yaml(const com2009_team69_2025::action::ExploreForward_GetResult_Response & msg)
{
  return com2009_team69_2025::action::to_yaml(msg);
}

template<>
inline const char * data_type<com2009_team69_2025::action::ExploreForward_GetResult_Response>()
{
  return "com2009_team69_2025::action::ExploreForward_GetResult_Response";
}

template<>
inline const char * name<com2009_team69_2025::action::ExploreForward_GetResult_Response>()
{
  return "com2009_team69_2025/action/ExploreForward_GetResult_Response";
}

template<>
struct has_fixed_size<com2009_team69_2025::action::ExploreForward_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<com2009_team69_2025::action::ExploreForward_Result>::value> {};

template<>
struct has_bounded_size<com2009_team69_2025::action::ExploreForward_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<com2009_team69_2025::action::ExploreForward_Result>::value> {};

template<>
struct is_message<com2009_team69_2025::action::ExploreForward_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<com2009_team69_2025::action::ExploreForward_GetResult>()
{
  return "com2009_team69_2025::action::ExploreForward_GetResult";
}

template<>
inline const char * name<com2009_team69_2025::action::ExploreForward_GetResult>()
{
  return "com2009_team69_2025/action/ExploreForward_GetResult";
}

template<>
struct has_fixed_size<com2009_team69_2025::action::ExploreForward_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<com2009_team69_2025::action::ExploreForward_GetResult_Request>::value &&
    has_fixed_size<com2009_team69_2025::action::ExploreForward_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<com2009_team69_2025::action::ExploreForward_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<com2009_team69_2025::action::ExploreForward_GetResult_Request>::value &&
    has_bounded_size<com2009_team69_2025::action::ExploreForward_GetResult_Response>::value
  >
{
};

template<>
struct is_service<com2009_team69_2025::action::ExploreForward_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<com2009_team69_2025::action::ExploreForward_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<com2009_team69_2025::action::ExploreForward_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "com2009_team69_2025/action/detail/explore_forward__traits.hpp"

namespace com2009_team69_2025
{

namespace action
{

inline void to_flow_style_yaml(
  const ExploreForward_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExploreForward_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExploreForward_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace com2009_team69_2025

namespace rosidl_generator_traits
{

[[deprecated("use com2009_team69_2025::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const com2009_team69_2025::action::ExploreForward_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  com2009_team69_2025::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use com2009_team69_2025::action::to_yaml() instead")]]
inline std::string to_yaml(const com2009_team69_2025::action::ExploreForward_FeedbackMessage & msg)
{
  return com2009_team69_2025::action::to_yaml(msg);
}

template<>
inline const char * data_type<com2009_team69_2025::action::ExploreForward_FeedbackMessage>()
{
  return "com2009_team69_2025::action::ExploreForward_FeedbackMessage";
}

template<>
inline const char * name<com2009_team69_2025::action::ExploreForward_FeedbackMessage>()
{
  return "com2009_team69_2025/action/ExploreForward_FeedbackMessage";
}

template<>
struct has_fixed_size<com2009_team69_2025::action::ExploreForward_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<com2009_team69_2025::action::ExploreForward_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<com2009_team69_2025::action::ExploreForward_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<com2009_team69_2025::action::ExploreForward_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<com2009_team69_2025::action::ExploreForward_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<com2009_team69_2025::action::ExploreForward>
  : std::true_type
{
};

template<>
struct is_action_goal<com2009_team69_2025::action::ExploreForward_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<com2009_team69_2025::action::ExploreForward_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<com2009_team69_2025::action::ExploreForward_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // COM2009_TEAM69_2025__ACTION__DETAIL__EXPLORE_FORWARD__TRAITS_HPP_
