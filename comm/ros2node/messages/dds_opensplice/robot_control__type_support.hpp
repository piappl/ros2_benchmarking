// generated from rosidl_typesupport_opensplice_cpp/resource/msg__type_support.hpp.template

#ifndef __robot_information_msgs__msg__dds_opensplice__robot_control__type_support__hpp__
#define __robot_information_msgs__msg__dds_opensplice__robot_control__type_support__hpp__

#include "robot_information_msgs/msg/robot_control__struct.hpp"
#include "robot_information_msgs/msg/dds_opensplice/ccpp_RobotControl_.h"
#include "robot_information_msgs/msg/dds_opensplice/visibility_control.h"

namespace DDS
{
class DomainParticipant;
class DataReader;
class DataWriter;
}  // namespace DDS

namespace robot_information_msgs
{

namespace msg
{

namespace typesupport_opensplice_cpp
{

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_robot_information_msgs
extern void register_type__RobotControl(
  DDS::DomainParticipant * participant,
  const char * type_name);

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_robot_information_msgs
extern void convert_ros_message_to_dds(
  const robot_information_msgs::msg::RobotControl& ros_message,
  robot_information_msgs::msg::dds_::RobotControl_& dds_message);

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_robot_information_msgs
extern void publish__RobotControl(
  DDS::DataWriter * topic_writer,
  const void * untyped_ros_message);

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_robot_information_msgs
extern void convert_dds_message_to_ros(
  const robot_information_msgs::msg::dds_::RobotControl_& dds_message,
  robot_information_msgs::msg::RobotControl& ros_message);

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_robot_information_msgs
extern bool take__RobotControl(
  DDS::DataReader * topic_reader,
  bool ignore_local_publications,
  void * untyped_ros_message,
  bool * taken);

}  // namespace typesupport_opensplice_cpp

}  // namespace msg

}  // namespace robot_information_msgs

#endif  // __robot_information_msgs__msg__dds_opensplice__robot_control__type_support__hpp__
