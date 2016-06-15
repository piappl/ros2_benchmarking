#ifndef MESSAGETYPES_H
#define MESSAGETYPES_H

#include <QMetaType>

namespace communication
{
    enum MessageType
    {
        MessageTypeRobotControl,
        MessageTypeRobotSensor,
        MessageTypeRobotAlarm
    };

    struct RobotControl
    {
        int id;
        int x;
        int y;
        int z;
    };

    struct RobotAlarm
    {
        int id;
        int alarm1;
        int alarm2;
    };

    struct RobotSensor
    {
        int id;
        std::string data;
    };
}

Q_DECLARE_METATYPE(communication::RobotControl)
Q_DECLARE_METATYPE(communication::RobotAlarm)
Q_DECLARE_METATYPE(communication::RobotSensor)

#endif //MESSAGETYPES_H
