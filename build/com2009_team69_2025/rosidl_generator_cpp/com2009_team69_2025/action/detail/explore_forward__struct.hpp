// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from com2009_team69_2025:action/ExploreForward.idl
// generated code does not contain a copyright notice

#ifndef COM2009_TEAM69_2025__ACTION__DETAIL__EXPLORE_FORWARD__STRUCT_HPP_
#define COM2009_TEAM69_2025__ACTION__DETAIL__EXPLORE_FORWARD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__com2009_team69_2025__action__ExploreForward_Goal __attribute__((deprecated))
#else
# define DEPRECATED__com2009_team69_2025__action__ExploreForward_Goal __declspec(deprecated)
#endif

namespace com2009_team69_2025
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ExploreForward_Goal_
{
  using Type = ExploreForward_Goal_<ContainerAllocator>;

  explicit ExploreForward_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fwd_velocity = 0.0f;
      this->stopping_distance = 0.0f;
    }
  }

  explicit ExploreForward_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fwd_velocity = 0.0f;
      this->stopping_distance = 0.0f;
    }
  }

  // field types and members
  using _fwd_velocity_type =
    float;
  _fwd_velocity_type fwd_velocity;
  using _stopping_distance_type =
    float;
  _stopping_distance_type stopping_distance;

  // setters for named parameter idiom
  Type & set__fwd_velocity(
    const float & _arg)
  {
    this->fwd_velocity = _arg;
    return *this;
  }
  Type & set__stopping_distance(
    const float & _arg)
  {
    this->stopping_distance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    com2009_team69_2025::action::ExploreForward_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const com2009_team69_2025::action::ExploreForward_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<com2009_team69_2025::action::ExploreForward_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<com2009_team69_2025::action::ExploreForward_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      com2009_team69_2025::action::ExploreForward_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<com2009_team69_2025::action::ExploreForward_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      com2009_team69_2025::action::ExploreForward_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<com2009_team69_2025::action::ExploreForward_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<com2009_team69_2025::action::ExploreForward_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<com2009_team69_2025::action::ExploreForward_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__com2009_team69_2025__action__ExploreForward_Goal
    std::shared_ptr<com2009_team69_2025::action::ExploreForward_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__com2009_team69_2025__action__ExploreForward_Goal
    std::shared_ptr<com2009_team69_2025::action::ExploreForward_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExploreForward_Goal_ & other) const
  {
    if (this->fwd_velocity != other.fwd_velocity) {
      return false;
    }
    if (this->stopping_distance != other.stopping_distance) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExploreForward_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExploreForward_Goal_

// alias to use template instance with default allocator
using ExploreForward_Goal =
  com2009_team69_2025::action::ExploreForward_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace com2009_team69_2025


#ifndef _WIN32
# define DEPRECATED__com2009_team69_2025__action__ExploreForward_Result __attribute__((deprecated))
#else
# define DEPRECATED__com2009_team69_2025__action__ExploreForward_Result __declspec(deprecated)
#endif

namespace com2009_team69_2025
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ExploreForward_Result_
{
  using Type = ExploreForward_Result_<ContainerAllocator>;

  explicit ExploreForward_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->total_distance_travelled = 0.0f;
      this->closest_obstacle = 0.0f;
    }
  }

  explicit ExploreForward_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->total_distance_travelled = 0.0f;
      this->closest_obstacle = 0.0f;
    }
  }

  // field types and members
  using _total_distance_travelled_type =
    float;
  _total_distance_travelled_type total_distance_travelled;
  using _closest_obstacle_type =
    float;
  _closest_obstacle_type closest_obstacle;

  // setters for named parameter idiom
  Type & set__total_distance_travelled(
    const float & _arg)
  {
    this->total_distance_travelled = _arg;
    return *this;
  }
  Type & set__closest_obstacle(
    const float & _arg)
  {
    this->closest_obstacle = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    com2009_team69_2025::action::ExploreForward_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const com2009_team69_2025::action::ExploreForward_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<com2009_team69_2025::action::ExploreForward_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<com2009_team69_2025::action::ExploreForward_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      com2009_team69_2025::action::ExploreForward_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<com2009_team69_2025::action::ExploreForward_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      com2009_team69_2025::action::ExploreForward_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<com2009_team69_2025::action::ExploreForward_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<com2009_team69_2025::action::ExploreForward_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<com2009_team69_2025::action::ExploreForward_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__com2009_team69_2025__action__ExploreForward_Result
    std::shared_ptr<com2009_team69_2025::action::ExploreForward_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__com2009_team69_2025__action__ExploreForward_Result
    std::shared_ptr<com2009_team69_2025::action::ExploreForward_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExploreForward_Result_ & other) const
  {
    if (this->total_distance_travelled != other.total_distance_travelled) {
      return false;
    }
    if (this->closest_obstacle != other.closest_obstacle) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExploreForward_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExploreForward_Result_

// alias to use template instance with default allocator
using ExploreForward_Result =
  com2009_team69_2025::action::ExploreForward_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace com2009_team69_2025


#ifndef _WIN32
# define DEPRECATED__com2009_team69_2025__action__ExploreForward_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__com2009_team69_2025__action__ExploreForward_Feedback __declspec(deprecated)
#endif

namespace com2009_team69_2025
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ExploreForward_Feedback_
{
  using Type = ExploreForward_Feedback_<ContainerAllocator>;

  explicit ExploreForward_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_distance_travelled = 0.0f;
    }
  }

  explicit ExploreForward_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_distance_travelled = 0.0f;
    }
  }

  // field types and members
  using _current_distance_travelled_type =
    float;
  _current_distance_travelled_type current_distance_travelled;

  // setters for named parameter idiom
  Type & set__current_distance_travelled(
    const float & _arg)
  {
    this->current_distance_travelled = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    com2009_team69_2025::action::ExploreForward_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const com2009_team69_2025::action::ExploreForward_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<com2009_team69_2025::action::ExploreForward_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<com2009_team69_2025::action::ExploreForward_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      com2009_team69_2025::action::ExploreForward_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<com2009_team69_2025::action::ExploreForward_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      com2009_team69_2025::action::ExploreForward_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<com2009_team69_2025::action::ExploreForward_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<com2009_team69_2025::action::ExploreForward_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<com2009_team69_2025::action::ExploreForward_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__com2009_team69_2025__action__ExploreForward_Feedback
    std::shared_ptr<com2009_team69_2025::action::ExploreForward_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__com2009_team69_2025__action__ExploreForward_Feedback
    std::shared_ptr<com2009_team69_2025::action::ExploreForward_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExploreForward_Feedback_ & other) const
  {
    if (this->current_distance_travelled != other.current_distance_travelled) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExploreForward_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExploreForward_Feedback_

// alias to use template instance with default allocator
using ExploreForward_Feedback =
  com2009_team69_2025::action::ExploreForward_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace com2009_team69_2025


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "com2009_team69_2025/action/detail/explore_forward__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__com2009_team69_2025__action__ExploreForward_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__com2009_team69_2025__action__ExploreForward_SendGoal_Request __declspec(deprecated)
#endif

namespace com2009_team69_2025
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ExploreForward_SendGoal_Request_
{
  using Type = ExploreForward_SendGoal_Request_<ContainerAllocator>;

  explicit ExploreForward_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit ExploreForward_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    com2009_team69_2025::action::ExploreForward_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const com2009_team69_2025::action::ExploreForward_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    com2009_team69_2025::action::ExploreForward_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const com2009_team69_2025::action::ExploreForward_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<com2009_team69_2025::action::ExploreForward_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<com2009_team69_2025::action::ExploreForward_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      com2009_team69_2025::action::ExploreForward_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<com2009_team69_2025::action::ExploreForward_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      com2009_team69_2025::action::ExploreForward_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<com2009_team69_2025::action::ExploreForward_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<com2009_team69_2025::action::ExploreForward_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<com2009_team69_2025::action::ExploreForward_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__com2009_team69_2025__action__ExploreForward_SendGoal_Request
    std::shared_ptr<com2009_team69_2025::action::ExploreForward_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__com2009_team69_2025__action__ExploreForward_SendGoal_Request
    std::shared_ptr<com2009_team69_2025::action::ExploreForward_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExploreForward_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExploreForward_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExploreForward_SendGoal_Request_

// alias to use template instance with default allocator
using ExploreForward_SendGoal_Request =
  com2009_team69_2025::action::ExploreForward_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace com2009_team69_2025


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__com2009_team69_2025__action__ExploreForward_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__com2009_team69_2025__action__ExploreForward_SendGoal_Response __declspec(deprecated)
#endif

namespace com2009_team69_2025
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ExploreForward_SendGoal_Response_
{
  using Type = ExploreForward_SendGoal_Response_<ContainerAllocator>;

  explicit ExploreForward_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit ExploreForward_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    com2009_team69_2025::action::ExploreForward_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const com2009_team69_2025::action::ExploreForward_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<com2009_team69_2025::action::ExploreForward_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<com2009_team69_2025::action::ExploreForward_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      com2009_team69_2025::action::ExploreForward_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<com2009_team69_2025::action::ExploreForward_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      com2009_team69_2025::action::ExploreForward_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<com2009_team69_2025::action::ExploreForward_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<com2009_team69_2025::action::ExploreForward_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<com2009_team69_2025::action::ExploreForward_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__com2009_team69_2025__action__ExploreForward_SendGoal_Response
    std::shared_ptr<com2009_team69_2025::action::ExploreForward_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__com2009_team69_2025__action__ExploreForward_SendGoal_Response
    std::shared_ptr<com2009_team69_2025::action::ExploreForward_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExploreForward_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExploreForward_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExploreForward_SendGoal_Response_

// alias to use template instance with default allocator
using ExploreForward_SendGoal_Response =
  com2009_team69_2025::action::ExploreForward_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace com2009_team69_2025

namespace com2009_team69_2025
{

namespace action
{

struct ExploreForward_SendGoal
{
  using Request = com2009_team69_2025::action::ExploreForward_SendGoal_Request;
  using Response = com2009_team69_2025::action::ExploreForward_SendGoal_Response;
};

}  // namespace action

}  // namespace com2009_team69_2025


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__com2009_team69_2025__action__ExploreForward_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__com2009_team69_2025__action__ExploreForward_GetResult_Request __declspec(deprecated)
#endif

namespace com2009_team69_2025
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ExploreForward_GetResult_Request_
{
  using Type = ExploreForward_GetResult_Request_<ContainerAllocator>;

  explicit ExploreForward_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit ExploreForward_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    com2009_team69_2025::action::ExploreForward_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const com2009_team69_2025::action::ExploreForward_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<com2009_team69_2025::action::ExploreForward_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<com2009_team69_2025::action::ExploreForward_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      com2009_team69_2025::action::ExploreForward_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<com2009_team69_2025::action::ExploreForward_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      com2009_team69_2025::action::ExploreForward_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<com2009_team69_2025::action::ExploreForward_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<com2009_team69_2025::action::ExploreForward_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<com2009_team69_2025::action::ExploreForward_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__com2009_team69_2025__action__ExploreForward_GetResult_Request
    std::shared_ptr<com2009_team69_2025::action::ExploreForward_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__com2009_team69_2025__action__ExploreForward_GetResult_Request
    std::shared_ptr<com2009_team69_2025::action::ExploreForward_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExploreForward_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExploreForward_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExploreForward_GetResult_Request_

// alias to use template instance with default allocator
using ExploreForward_GetResult_Request =
  com2009_team69_2025::action::ExploreForward_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace com2009_team69_2025


// Include directives for member types
// Member 'result'
// already included above
// #include "com2009_team69_2025/action/detail/explore_forward__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__com2009_team69_2025__action__ExploreForward_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__com2009_team69_2025__action__ExploreForward_GetResult_Response __declspec(deprecated)
#endif

namespace com2009_team69_2025
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ExploreForward_GetResult_Response_
{
  using Type = ExploreForward_GetResult_Response_<ContainerAllocator>;

  explicit ExploreForward_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit ExploreForward_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    com2009_team69_2025::action::ExploreForward_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const com2009_team69_2025::action::ExploreForward_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    com2009_team69_2025::action::ExploreForward_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const com2009_team69_2025::action::ExploreForward_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<com2009_team69_2025::action::ExploreForward_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<com2009_team69_2025::action::ExploreForward_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      com2009_team69_2025::action::ExploreForward_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<com2009_team69_2025::action::ExploreForward_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      com2009_team69_2025::action::ExploreForward_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<com2009_team69_2025::action::ExploreForward_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<com2009_team69_2025::action::ExploreForward_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<com2009_team69_2025::action::ExploreForward_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__com2009_team69_2025__action__ExploreForward_GetResult_Response
    std::shared_ptr<com2009_team69_2025::action::ExploreForward_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__com2009_team69_2025__action__ExploreForward_GetResult_Response
    std::shared_ptr<com2009_team69_2025::action::ExploreForward_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExploreForward_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExploreForward_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExploreForward_GetResult_Response_

// alias to use template instance with default allocator
using ExploreForward_GetResult_Response =
  com2009_team69_2025::action::ExploreForward_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace com2009_team69_2025

namespace com2009_team69_2025
{

namespace action
{

struct ExploreForward_GetResult
{
  using Request = com2009_team69_2025::action::ExploreForward_GetResult_Request;
  using Response = com2009_team69_2025::action::ExploreForward_GetResult_Response;
};

}  // namespace action

}  // namespace com2009_team69_2025


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "com2009_team69_2025/action/detail/explore_forward__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__com2009_team69_2025__action__ExploreForward_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__com2009_team69_2025__action__ExploreForward_FeedbackMessage __declspec(deprecated)
#endif

namespace com2009_team69_2025
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ExploreForward_FeedbackMessage_
{
  using Type = ExploreForward_FeedbackMessage_<ContainerAllocator>;

  explicit ExploreForward_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit ExploreForward_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    com2009_team69_2025::action::ExploreForward_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const com2009_team69_2025::action::ExploreForward_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    com2009_team69_2025::action::ExploreForward_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const com2009_team69_2025::action::ExploreForward_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<com2009_team69_2025::action::ExploreForward_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<com2009_team69_2025::action::ExploreForward_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      com2009_team69_2025::action::ExploreForward_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<com2009_team69_2025::action::ExploreForward_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      com2009_team69_2025::action::ExploreForward_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<com2009_team69_2025::action::ExploreForward_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<com2009_team69_2025::action::ExploreForward_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<com2009_team69_2025::action::ExploreForward_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__com2009_team69_2025__action__ExploreForward_FeedbackMessage
    std::shared_ptr<com2009_team69_2025::action::ExploreForward_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__com2009_team69_2025__action__ExploreForward_FeedbackMessage
    std::shared_ptr<com2009_team69_2025::action::ExploreForward_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExploreForward_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExploreForward_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExploreForward_FeedbackMessage_

// alias to use template instance with default allocator
using ExploreForward_FeedbackMessage =
  com2009_team69_2025::action::ExploreForward_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace com2009_team69_2025

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace com2009_team69_2025
{

namespace action
{

struct ExploreForward
{
  /// The goal message defined in the action definition.
  using Goal = com2009_team69_2025::action::ExploreForward_Goal;
  /// The result message defined in the action definition.
  using Result = com2009_team69_2025::action::ExploreForward_Result;
  /// The feedback message defined in the action definition.
  using Feedback = com2009_team69_2025::action::ExploreForward_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = com2009_team69_2025::action::ExploreForward_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = com2009_team69_2025::action::ExploreForward_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = com2009_team69_2025::action::ExploreForward_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct ExploreForward ExploreForward;

}  // namespace action

}  // namespace com2009_team69_2025

#endif  // COM2009_TEAM69_2025__ACTION__DETAIL__EXPLORE_FORWARD__STRUCT_HPP_
