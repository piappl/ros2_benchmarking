#include "RobotStatus_SplDcps.h"
#include "ccpp_RobotStatus_.h"
#include "dds_type_aliases.h"

const char *
__robot_information_msgs_msg_dds__RobotStatus___name(void)
{
    return (const char*)"robot_information_msgs::msg::dds_::RobotStatus_";
}

const char *
__robot_information_msgs_msg_dds__RobotStatus___keys(void)
{
    return (const char*)"";
}

#include <v_kernel.h>
#include <v_topic.h>
#include <os_stdlib.h>
#include <string.h>
#include <os_report.h>

c_bool
__robot_information_msgs_msg_dds__RobotStatus___copyIn(
    c_base base,
    struct ::robot_information_msgs::msg::dds_::RobotStatus_ *from,
    struct _robot_information_msgs_msg_dds__RobotStatus_ *to)
{
    c_bool result = OS_C_TRUE;
    (void) base;

    to->battery_ = (c_octet)from->battery_;
    to->turtle_factor_ = (c_octet)from->turtle_factor_;
    to->battery_charging_ = (c_bool)from->battery_charging_;
    to->drive_reversed_ = (c_bool)from->drive_reversed_;
    to->emergency_active_ = (c_bool)from->emergency_active_;
    to->brake_active_ = (c_bool)from->brake_active_;
    return result;
}

void
__robot_information_msgs_msg_dds__RobotStatus___copyOut(
    void *_from,
    void *_to)
{
    struct _robot_information_msgs_msg_dds__RobotStatus_ *from = (struct _robot_information_msgs_msg_dds__RobotStatus_ *)_from;
    struct ::robot_information_msgs::msg::dds_::RobotStatus_ *to = (struct ::robot_information_msgs::msg::dds_::RobotStatus_ *)_to;
    to->battery_ = (::DDS::Octet)from->battery_;
    to->turtle_factor_ = (::DDS::Octet)from->turtle_factor_;
    to->battery_charging_ = (::DDS::Boolean)(from->battery_charging_ != 0);
    to->drive_reversed_ = (::DDS::Boolean)(from->drive_reversed_ != 0);
    to->emergency_active_ = (::DDS::Boolean)(from->emergency_active_ != 0);
    to->brake_active_ = (::DDS::Boolean)(from->brake_active_ != 0);
}

