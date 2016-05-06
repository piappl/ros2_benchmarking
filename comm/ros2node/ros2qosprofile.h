#ifndef ROS2QOSPROFILE_H
#define ROS2QOSPROFILE_H

#include <rmw/qos_profiles.h>
#include <common/messagetypes.h>

namespace roscommunication
{
    class Ros2QoSProfile
    {
    public:
        //TODO - receiver vs subscriber QoS
        static rmw_qos_profile_t getProfile(communication::MessageType n);
    };
}

#endif // ROS2QOSPROFILE_H
