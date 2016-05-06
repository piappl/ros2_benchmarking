// generated from rosidl_generator_c/resource/msg__struct.h.template
// generated code does not contain a copyright notice

#ifndef ROBOT_INFORMATION_MSGS__MSG__ROBOT_CONTROL__STRUCT_H_
#define ROBOT_INFORMATION_MSGS__MSG__ROBOT_CONTROL__STRUCT_H_

#if __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/// Struct of message robot_information_msgs/RobotControl
typedef struct robot_information_msgs__msg__RobotControl
{
  uint8_t turtle;
  bool drive_reversed;
  bool emergency_active;
} robot_information_msgs__msg__RobotControl;

/// Struct for an array of messages
typedef struct robot_information_msgs__msg__RobotControl__Array
{
  robot_information_msgs__msg__RobotControl * data;
  size_t size;
  size_t capacity;
} robot_information_msgs__msg__RobotControl__Array;

#if __cplusplus
}
#endif

#endif  // ROBOT_INFORMATION_MSGS__MSG__ROBOT_CONTROL__STRUCT_H_
