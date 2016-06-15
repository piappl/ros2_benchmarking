#ifndef ROS2MESSAGETOVARIANT_H
#define ROS2MESSAGETOVARIANT_H

#include <QVariant>
#include <common/logging.h>
#include <common/messagetypes.h>
#include "ros2messages.h"

namespace roscommunication
{
    template <typename T>
    QVariant ros2MessageToVariant(T message)
    {
        Q_UNUSED(message);
        //ERROR
        return QVariant(false);
    }

    template <>
    QVariant ros2MessageToVariant<messages::msg::RobotControl::SharedPtr>(messages::msg::RobotControl::SharedPtr msg)
    {
        debug(LOG_BENCHMARK, "RECEIVED RobotControl", "id=%d, size=%lu", msg->id, sizeof(messages::msg::RobotControl));
        communication::RobotControl control;
        control.id = msg->id;
        control.x = msg->x;
        control.y = msg->y;
        control.z = msg->z;
        return QVariant::fromValue(control);
    }

    template <>
    QVariant ros2MessageToVariant<messages::msg::RobotAlarm::SharedPtr>(messages::msg::RobotAlarm::SharedPtr msg)
    {
        debug(LOG_BENCHMARK, "RECEIVED RobotAlarm", "id=%u, size=%lu", msg->id, sizeof(messages::msg::RobotAlarm));
        communication::RobotAlarm alarm;
        alarm.id = msg->id;
        alarm.alarm1 = msg->alarm1;
        alarm.alarm2 = msg->alarm2;
        return QVariant::fromValue(alarm);
    }

    template <>
    QVariant ros2MessageToVariant<messages::msg::RobotSensor::SharedPtr>(messages::msg::RobotSensor::SharedPtr msg)
    {
        debug(LOG_BENCHMARK, "RECEIVED RobotSensor", "id=%d, size=%lu", msg->id, msg->data.size());
        communication::RobotSensor sensor;
        sensor.id = msg->id;
        sensor.data = msg->data;
        return QVariant::fromValue(sensor);
    }
}
#endif // ROS2MESSAGETOVARIANT_H
