#ifndef DDSNODE_H
#define DDSNODE_H

#include <QSharedPointer>
#include <common/messagetypes.h>
#include <common/nodeinterface.h>
#include "ddsinclude.h"
#include "ddspublisher.h"
#include "ddssubscriber.h"
#include "ddstopics.h"

//TODO - prototype version - refactor
namespace ddscommunication
{
    class DDSNode : public communication::NodeInterface
    {
    public:
        DDSNode(int domainID);
        void start() {} //TODO

        void advertise(communication::MessageType type);
        void subscribe(communication::MessageType type);
        void unsubscribe(communication::MessageType type);

        void publishCmdVel(communication::MoveBase cmdVel);
        void publishRobotStatus(communication::RobotStatus status);
        void publishRobotControl(communication::RobotControl control);
        void publishByteMessage(int size);

    private:
        Participant mDomainParticipant;
        DDSTopics mTopics;
        DDSPublisher mPublisher;
        DDSSubscriber mSubscriber;
    };
    typedef QSharedPointer<DDSNode> DDSNodeInterfacePtr;
}

#endif //DDSNODE_H
