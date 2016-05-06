#ifndef ROBOTCONTROL_SPLTYPES_H
#define ROBOTCONTROL_SPLTYPES_H

#include "ccpp_RobotControl_.h"

#include <c_base.h>
#include <c_misc.h>
#include <c_sync.h>
#include <c_collection.h>
#include <c_field.h>

extern c_metaObject __RobotControl__robot_information_msgs__load (c_base base);

extern c_metaObject __RobotControl__robot_information_msgs_msg__load (c_base base);

extern c_metaObject __RobotControl__robot_information_msgs_msg_dds___load (c_base base);

extern c_metaObject __robot_information_msgs_msg_dds__RobotControl___load (c_base base);
extern const char * __robot_information_msgs_msg_dds__RobotControl___keys (void);
extern const char * __robot_information_msgs_msg_dds__RobotControl___name (void);
struct _robot_information_msgs_msg_dds__RobotControl_ ;
extern  c_bool __robot_information_msgs_msg_dds__RobotControl___copyIn(c_base base, struct robot_information_msgs::msg::dds_::RobotControl_ *from, struct _robot_information_msgs_msg_dds__RobotControl_ *to);
extern  void __robot_information_msgs_msg_dds__RobotControl___copyOut(void *_from, void *_to);
struct _robot_information_msgs_msg_dds__RobotControl_ {
    c_octet turtle_;
    c_bool drive_reversed_;
    c_bool emergency_active_;
};

#endif
