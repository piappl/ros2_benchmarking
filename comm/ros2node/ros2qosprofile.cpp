#include "ros2qosprofile.h"
#include <common/logging.h>

using namespace roscommunication;
using namespace communication;

namespace
{
    static const rmw_qos_profile_t rmw_qos_profile_caps =
    {
      RMW_QOS_POLICY_KEEP_ALL_HISTORY,
      100,
      RMW_QOS_POLICY_RELIABLE,
      RMW_QOS_POLICY_TRANSIENT_LOCAL_DURABILITY
    };
}

rmw_qos_profile_t Ros2QoSProfile::getProfile(MessageType n)
{
    switch (n)
    {
        case MessageTypeBytes:
        case MessageTypeCmdVel:
            return rmw_qos_profile_sensor_data;
        case MessageTypeRobotControl:
        case MessageTypeRobotStatus:
            return rmw_qos_profile_services_default;
        default:
            debug(LOG_ERROR, "Ros2QoSProfile", "Profile unspecified for type, falling to default");
            return rmw_qos_profile_default;
    }
}
