#ifndef NODEINTERFACE_H
#define NODEINTERFACE_H

#include <QSharedPointer>
#include <common/messagetypes.h>

namespace communication
{
    class NodeInterface
    {
    public:
        virtual ~NodeInterface() {}

        virtual void start() = 0;

        virtual void advertise(communication::MessageType type) = 0;
        virtual void subscribe(communication::MessageType type) = 0;
        virtual void unsubscribe(communication::MessageType type) = 0;

        virtual void publishRobotControl(communication::RobotControl control) = 0;
        virtual void publishRobotAlarm(communication::RobotAlarm alarm) = 0;
        virtual void publishRobotSensor(communication::RobotSensor sensor) = 0;
    };
    typedef QSharedPointer<NodeInterface> NodeInterfacePtr;
}

#endif //NODEINTERFACE_H
