// generated from rosidl_generator_cpp/resource/msg__struct.hpp.template
// generated code does not contain a copyright notice

#ifndef ROBOT_INFORMATION_MSGS__MSG__ROBOT_CONTROL__STRUCT_HPP_
#define ROBOT_INFORMATION_MSGS__MSG__ROBOT_CONTROL__STRUCT_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

// include message dependencies

#ifndef _WIN32
# define DEPRECATED_robot_information_msgs_msg_RobotControl __attribute__((deprecated))
#else
# define DEPRECATED_robot_information_msgs_msg_RobotControl __declspec(deprecated)
#endif

namespace robot_information_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobotControl_
{
  using Type = RobotControl_<ContainerAllocator>;

  RobotControl_()
  {
  }
  explicit RobotControl_(const ContainerAllocator & _alloc)
  {
    (void)_alloc;
  }

  // field types and members
  using _turtle_type =
      uint8_t;
  _turtle_type turtle;
  using _drive_reversed_type =
      bool;
  _drive_reversed_type drive_reversed;
  using _emergency_active_type =
      bool;
  _emergency_active_type emergency_active;

  // setters for named parameter idiom
  Type * set__turtle(
    const uint8_t & _arg)
  {
    this->turtle = _arg;
    return this;
  }
  Type * set__drive_reversed(
    const bool & _arg)
  {
    this->drive_reversed = _arg;
    return this;
  }
  Type * set__emergency_active(
    const bool & _arg)
  {
    this->emergency_active = _arg;
    return this;
  }

  // constants

  // pointer types
  using RawPtr =
    robot_information_msgs::msg::RobotControl_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_information_msgs::msg::RobotControl_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_information_msgs::msg::RobotControl_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_information_msgs::msg::RobotControl_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
    robot_information_msgs::msg::RobotControl_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_information_msgs::msg::RobotControl_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
    robot_information_msgs::msg::RobotControl_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_information_msgs::msg::RobotControl_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_information_msgs::msg::RobotControl_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_information_msgs::msg::RobotControl_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED_robot_information_msgs_msg_RobotControl
    std::shared_ptr<robot_information_msgs::msg::RobotControl_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED_robot_information_msgs_msg_RobotControl
    std::shared_ptr<robot_information_msgs::msg::RobotControl_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotControl_ & other) const
  {
    if (this->turtle != other.turtle) {
      return false;
    }
    if (this->drive_reversed != other.drive_reversed) {
      return false;
    }
    if (this->emergency_active != other.emergency_active) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotControl_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotControl_

// alias to use template instance with default allocator
using RobotControl =
    robot_information_msgs::msg::RobotControl_<std::allocator<void>>;

// constants requiring out of line definition

}  // namespace msg

}  // namespace robot_information_msgs

#endif  // ROBOT_INFORMATION_MSGS__MSG__ROBOT_CONTROL__STRUCT_HPP_
