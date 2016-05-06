#ifndef MESSAGETYPES_H
#define MESSAGETYPES_H

#include <QMetaType>

namespace communication
{
    enum MessageType
    {
        MessageTypeCmdVel,
        MessageTypeBytes,
        MessageTypeRobotControl,
        MessageTypeRobotStatus
    };

    struct MoveBase
    {
        int id;
        int x;
        int y;
        int z;
    };

    struct RobotControl
    {
        int id;
        int field1;
        int field2;
        int field3;
        int field4;
        int field5;
        int field6;
        int field7;
    };

    struct RobotStatus : RobotControl
    {
    };
}

Q_DECLARE_METATYPE(communication::MoveBase)
Q_DECLARE_METATYPE(communication::RobotControl)
Q_DECLARE_METATYPE(communication::RobotStatus)

#endif //MESSAGETYPES_H
