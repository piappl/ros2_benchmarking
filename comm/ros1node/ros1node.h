#ifndef ROS1NODE_H
#define ROS1NODE_H

#include <QString>
#include <common/messagetypes.h>
#include <common/nodeinterface.h>

namespace roscommunication
{
    class Ros1NodeImpl;

    class Ros1Node : public communication::NodeInterface
    {
    public:
        Ros1Node(QString name);
        ~Ros1Node();

        void start() {}
        void advertise(communication::MessageType notification);
        void subscribe(communication::MessageType t);
        void unsubscribe(communication::MessageType t);

        void publishCmdVel(communication::MoveBase cmdVel);
        void publishRobotStatus(communication::RobotStatus status);
        void publishRobotControl(communication::RobotControl control);
        void publishByteMessage(int size);

    private:
        Ros1NodeImpl *d;
    };
}

#endif // ROS1NODE_H
