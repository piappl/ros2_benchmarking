// generated from rosidl_generator_c/resource/msg__struct.h.template
// generated code does not contain a copyright notice

#ifndef ROBOT_INFORMATION_MSGS__MSG__ROBOT_STATUS__STRUCT_H_
#define ROBOT_INFORMATION_MSGS__MSG__ROBOT_STATUS__STRUCT_H_

#if __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/// Struct of message robot_information_msgs/RobotStatus
typedef struct robot_information_msgs__msg__RobotStatus
{
  int8_t battery;
  uint8_t turtle_factor;
  bool battery_charging;
  bool drive_reversed;
  bool emergency_active;
  bool brake_active;
} robot_information_msgs__msg__RobotStatus;

/// Struct for an array of messages
typedef struct robot_information_msgs__msg__RobotStatus__Array
{
  robot_information_msgs__msg__RobotStatus * data;
  size_t size;
  size_t capacity;
} robot_information_msgs__msg__RobotStatus__Array;

#if __cplusplus
}
#endif

#endif  // ROBOT_INFORMATION_MSGS__MSG__ROBOT_STATUS__STRUCT_H_
