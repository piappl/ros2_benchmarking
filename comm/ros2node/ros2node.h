#ifndef ROS2NODE_H
#define ROS2NODE_H

#include <QString>
#include <common/messagetypes.h>
#include <common/rosnodeinterface.h>

namespace roscommunication
{
    class Ros2NodeImpl;

    class Ros2Node : public RosNodeInterface
    {
    public:
        Ros2Node(QString name);
        ~Ros2Node();

        void start();

        void advertise(communication::MessageType notification);
        void subscribe(communication::MessageType notification, bool subscribe = true);

        void publishCmdVel(communication::MoveBase cmdVel);
        void publishRobotStatus(communication::RobotStatus status);
        void publishRobotControl(communication::RobotControl control);
        void publishByteMessage(int size);

        Ros2NodeImpl *d;
    };
}

#endif // ROS2NODE_H
