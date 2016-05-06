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
    QVariant ros2MessageToVariant<geometry_msgs::msg::Transform::SharedPtr>(
            geometry_msgs::msg::Transform::SharedPtr msg)
    {
        debug(LOG_BENCHMARK, "RECEIVED cmd_vel", "id=%d, x=%lf, turn=%lf",
              (int)msg->translation.z, msg->translation.x, msg->rotation.z);

        communication::MoveBase command;
        command.x = msg->translation.x;
        command.z = msg->rotation.z;
        command.id = msg->translation.z;
        return QVariant::fromValue(command);
    }

    template <>
    QVariant ros2MessageToVariant<robot_information_msgs::msg::RobotControl::SharedPtr>(
            robot_information_msgs::msg::RobotControl::SharedPtr msg)
    {
        debug(LOG_BENCHMARK, "RECEIVED robot_control", "id=%u", msg->emergency_active);

        communication::RobotControl control;
        control.field1 = msg->drive_reversed;
        control.id = msg->emergency_active; //TODO
        control.field2 = msg->turtle;
        return QVariant::fromValue(control);
    }

    template <>
    QVariant ros2MessageToVariant<robot_information_msgs::msg::RobotStatus::SharedPtr>(
            robot_information_msgs::msg::RobotStatus::SharedPtr msg)
    {
        communication::RobotStatus status;
        status.field1 = msg->brake_active;
        status.field2 = msg->battery;
        status.field3 = msg->battery_charging;
        status.field4 = msg->drive_reversed;
        status.id = msg->emergency_active; //TODO
        status.field5 = msg->turtle_factor;
        return QVariant::fromValue(status);
    }

    template <>
    QVariant ros2MessageToVariant<std_msgs::msg::ByteMultiArray::SharedPtr>(
            std_msgs::msg::ByteMultiArray::SharedPtr msg)
    {   //TODO
        communication::CommunicationUtils::dumpToHex((const int8_t*)msg->data.data(), msg->data.size(), "Ros2Publisher::received");
        return QVariant::fromValue(msg->data.size());
    }
}
#endif // ROS2MESSAGETOVARIANT_H
