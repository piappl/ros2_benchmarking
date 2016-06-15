#ifndef ROS1NODE_H
#define ROS1NODE_H

#include <QString>
#include <common/messagetypes.h>
#include <common/nodeinterface.h>
#include <common/settings.h>

namespace roscommunication
{
    class Ros1NodeImpl;

    class Ros1Node : public communication::NodeInterface
    {
    public:
        Ros1Node(communication::Settings settings);
        ~Ros1Node();

        void start() {}
        void advertise(communication::MessageType notification);
        void subscribe(communication::MessageType t);
        void unsubscribe(communication::MessageType t);

        void publishRobotControl(communication::RobotControl control);
        void publishRobotAlarm(communication::RobotAlarm alarm);
        void publishRobotSensor(communication::RobotSensor sensor);

    private:
        Ros1NodeImpl *d;
    };
}

#endif // ROS1NODE_H
