#include "RobotControl_SplDcps.h"
#include "ccpp_RobotControl_.h"
#include "dds_type_aliases.h"

const char *
__robot_information_msgs_msg_dds__RobotControl___name(void)
{
    return (const char*)"robot_information_msgs::msg::dds_::RobotControl_";
}

const char *
__robot_information_msgs_msg_dds__RobotControl___keys(void)
{
    return (const char*)"";
}

#include <v_kernel.h>
#include <v_topic.h>
#include <os_stdlib.h>
#include <string.h>
#include <os_report.h>

c_bool
__robot_information_msgs_msg_dds__RobotControl___copyIn(
    c_base base,
    struct ::robot_information_msgs::msg::dds_::RobotControl_ *from,
    struct _robot_information_msgs_msg_dds__RobotControl_ *to)
{
    c_bool result = OS_C_TRUE;
    (void) base;

    to->turtle_ = (c_octet)from->turtle_;
    to->drive_reversed_ = (c_bool)from->drive_reversed_;
    to->emergency_active_ = (c_bool)from->emergency_active_;
    return result;
}

void
__robot_information_msgs_msg_dds__RobotControl___copyOut(
    void *_from,
    void *_to)
{
    struct _robot_information_msgs_msg_dds__RobotControl_ *from = (struct _robot_information_msgs_msg_dds__RobotControl_ *)_from;
    struct ::robot_information_msgs::msg::dds_::RobotControl_ *to = (struct ::robot_information_msgs::msg::dds_::RobotControl_ *)_to;
    to->turtle_ = (::DDS::Octet)from->turtle_;
    to->drive_reversed_ = (::DDS::Boolean)(from->drive_reversed_ != 0);
    to->emergency_active_ = (::DDS::Boolean)(from->emergency_active_ != 0);
}

