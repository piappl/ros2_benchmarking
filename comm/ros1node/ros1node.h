#ifndef ROS1NODE_H
#define ROS1NODE_H

#include <QString>
#include <common/messagetypes.h>

namespace roscommunication
{
    class Ros1NodeImpl;

    class Ros1Node
    {
    public:
        Ros1Node(QString name);
        ~Ros1Node();

        void advertise(communication::MessageType notification);
        void subscribe(communication::MessageType n, bool sub = true);

        void publishCmdVel(communication::MoveBase cmdVel);
        void publishRobotStatus(communication::RobotStatus status);
        void publishRobotControl(communication::RobotControl control);
        void publishByteMessage(int size);

        Ros1NodeImpl *d;
    };
}

#endif // ROS1NODE_H
