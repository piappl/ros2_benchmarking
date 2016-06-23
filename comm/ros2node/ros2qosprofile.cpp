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
        RMW_QOS_POLICY_TRANSIENT_LOCAL_DURABILITY
    };

    static const rmw_qos_profile_t rmw_qos_profile_control =
    {
        RMW_QOS_POLICY_KEEP_LAST_HISTORY,
        10,
        RMW_QOS_POLICY_RELIABLE,
        RMW_QOS_POLICY_VOLATILE_DURABILITY
    };

    static const rmw_qos_profile_t rmw_qos_profile_sensor =
    {
        RMW_QOS_POLICY_KEEP_LAST_HISTORY,
        0,
        RMW_QOS_POLICY_BEST_EFFORT,
        RMW_QOS_POLICY_VOLATILE_DURABILITY
    };
}

rmw_qos_profile_t Ros2QoSProfile::getProfile(QoSProfile p)
{
    switch (p)
    {
        case QoSProfileAlarm:
            return rmw_qos_profile_alarm;
        case QoSProfileSensor:
            return rmw_qos_profile_sensor;
        case QoSProfileControl:
            return rmw_qos_profile_control;
        default:
            debug(LOG_ERROR, "Ros2QoSProfile", "Profile unspecified for type, falling to default");
            return rmw_qos_profile_default;
    }
}

std::string Ros2QoSProfile::profileDescription(QoSProfile p)
{
    auto profile = Ros2QoSProfile::getProfile(p);
    std::string description;
    switch (profile.history)
    {
        case RMW_QOS_POLICY_KEEP_ALL_HISTORY:
            description.append("H: KEEP_ALL, ");
            break;
        case RMW_QOS_POLICY_KEEP_LAST_HISTORY:
            description.append("H: KEEP_LAST=" + std::to_string(profile.depth)+ ", ");
            break;
    }
    switch (profile.reliability)
    {
        case RMW_QOS_POLICY_RELIABLE:
            description.append("R: RELIABLE, ");
            break;
        case RMW_QOS_POLICY_BEST_EFFORT:
            description.append("R: BEST_EFFORT, ");
            break;
    };
    switch (profile.durability)
    {
        case RMW_QOS_POLICY_TRANSIENT_LOCAL_DURABILITY:
            description.append("V: TRANSIENT");
            break;
        case RMW_QOS_POLICY_VOLATILE_DURABILITY:
            description.append("V: VOLATILE");
            break;
    };
    return description;
}

