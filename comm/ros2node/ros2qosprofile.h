#ifndef ROS2QOSPROFILE_H
#define ROS2QOSPROFILE_H

#include <rmw/qos_profiles.h>
#include <common/messagetypes.h>
#include <common/qosprofiles.h>

namespace roscommunication
{
    class Ros2QoSProfile
    {
    public:
        static rmw_qos_profile_t getProfile(communication::QoSProfile qos);
    };
}

#endif // ROS2QOSPROFILE_H
