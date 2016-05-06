// generated from rosidl_generator_cpp/resource/msg__struct.hpp.template
// generated code does not contain a copyright notice

#ifndef ROBOT_INFORMATION_MSGS__MSG__ROBOT_STATUS__STRUCT_HPP_
#define ROBOT_INFORMATION_MSGS__MSG__ROBOT_STATUS__STRUCT_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

// include message dependencies

#ifndef _WIN32
# define DEPRECATED_robot_information_msgs_msg_RobotStatus __attribute__((deprecated))
#else
# define DEPRECATED_robot_information_msgs_msg_RobotStatus __declspec(deprecated)
#endif

namespace robot_information_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobotStatus_
{
  using Type = RobotStatus_<ContainerAllocator>;

  RobotStatus_()
  {
  }
  explicit RobotStatus_(const ContainerAllocator & _alloc)
  {
    (void)_alloc;
  }

  // field types and members
  using _battery_type =
      int8_t;
  _battery_type battery;
  using _turtle_factor_type =
      uint8_t;
  _turtle_factor_type turtle_factor;
  using _battery_charging_type =
      bool;
  _battery_charging_type battery_charging;
  using _drive_reversed_type =
      bool;
  _drive_reversed_type drive_reversed;
  using _emergency_active_type =
      bool;
  _emergency_active_type emergency_active;
  using _brake_active_type =
      bool;
  _brake_active_type brake_active;

  // setters for named parameter idiom
  Type * set__battery(
    const int8_t & _arg)
  {
    this->battery = _arg;
    return this;
  }
  Type * set__turtle_factor(
    const uint8_t & _arg)
  {
    this->turtle_factor = _arg;
    return this;
  }
  Type * set__battery_charging(
    const bool & _arg)
  {
    this->battery_charging = _arg;
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
  Type * set__brake_active(
    const bool & _arg)
  {
    this->brake_active = _arg;
    return this;
  }

  // constants

  // pointer types
  using RawPtr =
    robot_information_msgs::msg::RobotStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_information_msgs::msg::RobotStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_information_msgs::msg::RobotStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_information_msgs::msg::RobotStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
    robot_information_msgs::msg::RobotStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_information_msgs::msg::RobotStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
    robot_information_msgs::msg::RobotStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_information_msgs::msg::RobotStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_information_msgs::msg::RobotStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_information_msgs::msg::RobotStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED_robot_information_msgs_msg_RobotStatus
    std::shared_ptr<robot_information_msgs::msg::RobotStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED_robot_information_msgs_msg_RobotStatus
    std::shared_ptr<robot_information_msgs::msg::RobotStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotStatus_ & other) const
  {
    if (this->battery != other.battery) {
      return false;
    }
    if (this->turtle_factor != other.turtle_factor) {
      return false;
    }
    if (this->battery_charging != other.battery_charging) {
      return false;
    }
    if (this->drive_reversed != other.drive_reversed) {
      return false;
    }
    if (this->emergency_active != other.emergency_active) {
      return false;
    }
    if (this->brake_active != other.brake_active) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotStatus_

// alias to use template instance with default allocator
using RobotStatus =
    robot_information_msgs::msg::RobotStatus_<std::allocator<void>>;

// constants requiring out of line definition

}  // namespace msg

}  // namespace robot_information_msgs

#endif  // ROBOT_INFORMATION_MSGS__MSG__ROBOT_STATUS__STRUCT_HPP_
