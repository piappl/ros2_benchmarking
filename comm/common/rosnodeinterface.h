#ifndef ROSNODEINTERFACE_H
#define ROSNODEINTERFACE_H

#include <QSharedPointer>
#include <common/messagetypes.h>

namespace roscommunication
{
    class RosNodeInterface
    {
    public:
        virtual ~RosNodeInterface() {}

        virtual void start() = 0;

        virtual void advertise(communication::MessageType notification) = 0;
        virtual void subscribe(communication::MessageType notification, bool subscribe = true) = 0;

        virtual void publishCmdVel(communication::MoveBase cmdVel) = 0;
        virtual void publishRobotStatus(communication::RobotStatus status) = 0;
        virtual void publishRobotControl(communication::RobotControl control) = 0;
        virtual void publishByteMessage(int size) = 0;
    };
    typedef QSharedPointer<RosNodeInterface> RosNodeInterfacePtr;
}

#endif //ROSNODEINTERFACE_H
