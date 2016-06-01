#ifndef ROS2NODE_H
#define ROS2NODE_H

#include <QString>
#include <common/messagetypes.h>
#include <common/nodeinterface.h>
#include <common/settings.h>

namespace roscommunication
{
    class Ros2NodeImpl;

    class Ros2Node : public communication::NodeInterface
    {
    public:
        Ros2Node(communication::Settings settings);
        ~Ros2Node();

        void start();
        void advertise(communication::MessageType notification);
        void subscribe(communication::MessageType notification);
        void unsubscribe(communication::MessageType notification);

        void publishCmdVel(communication::MoveBase cmdVel);
        void publishRobotStatus(communication::RobotStatus status);
        void publishRobotControl(communication::RobotControl control);
        void publishByteMessage(int size);

    private:
        Ros2NodeImpl *d;
    };
}

#endif // ROS2NODE_H
