#include "ros2qosprofile.h"
#include <common/logging.h>

using namespace roscommunication;
using namespace communication;

namespace
{
    static const rmw_qos_profile_t rmw_qos_profile_alarm =
    {
      RMW_QOS_POLICY_KEEP_ALL_HISTORY,
      100,
      RMW_QOS_POLICY_RELIABLE,
      RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT
    };

    static const rmw_qos_profile_t rmw_qos_profile_control =
    {
      RMW_QOS_POLICY_KEEP_LAST_HISTORY,
      10,
      RMW_QOS_POLICY_BEST_EFFORT,
      RMW_QOS_POLICY_VOLATILE_DURABILITY
    };

    static const rmw_qos_profile_t rmw_qos_profile_status = rmw_qos_profile_control;
}

rmw_qos_profile_t Ros2QoSProfile::getProfile(QoSProfile p)
{
    switch (p)
    {   //TODO - experiment here
        case QoSProfileAlarm:
            return rmw_qos_profile_alarm;
        case QoSProfileSensor:
            return rmw_qos_profile_sensor_data;
        case QoSProfileControl:
            return rmw_qos_profile_control;
        case QoSProfileStatus:
            return rmw_qos_profile_status;
        default:
            debug(LOG_ERROR, "Ros2QoSProfile", "Profile unspecified for type, falling to default");
            return rmw_qos_profile_default;
    }
}
