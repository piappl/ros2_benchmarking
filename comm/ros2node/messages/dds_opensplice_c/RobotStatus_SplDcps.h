#ifndef ROBOTSTATUS_SPLTYPES_H
#define ROBOTSTATUS_SPLTYPES_H

#include "ccpp_RobotStatus_.h"

#include <c_base.h>
#include <c_misc.h>
#include <c_sync.h>
#include <c_collection.h>
#include <c_field.h>

extern c_metaObject __RobotStatus__robot_information_msgs__load (c_base base);

extern c_metaObject __RobotStatus__robot_information_msgs_msg__load (c_base base);

extern c_metaObject __RobotStatus__robot_information_msgs_msg_dds___load (c_base base);

extern c_metaObject __robot_information_msgs_msg_dds__RobotStatus___load (c_base base);
extern const char * __robot_information_msgs_msg_dds__RobotStatus___keys (void);
extern const char * __robot_information_msgs_msg_dds__RobotStatus___name (void);
struct _robot_information_msgs_msg_dds__RobotStatus_ ;
extern  c_bool __robot_information_msgs_msg_dds__RobotStatus___copyIn(c_base base, struct robot_information_msgs::msg::dds_::RobotStatus_ *from, struct _robot_information_msgs_msg_dds__RobotStatus_ *to);
extern  void __robot_information_msgs_msg_dds__RobotStatus___copyOut(void *_from, void *_to);
struct _robot_information_msgs_msg_dds__RobotStatus_ {
    c_octet battery_;
    c_octet turtle_factor_;
    c_bool battery_charging_;
    c_bool drive_reversed_;
    c_bool emergency_active_;
    c_bool brake_active_;
};

#endif
