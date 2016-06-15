#ifndef DDSNODE_H
#define DDSNODE_H

#include <QSharedPointer>
#include <common/messagetypes.h>
#include <common/nodeinterface.h>
#include <common/settings.h>
#include "ddsinclude.h"
#include "ddspublisher.h"
#include "ddssubscriber.h"
#include "ddstopics.h"

//TODO - prototype version - refactor
namespace communication
{
    class DDSNode : public communication::NodeInterface
    {
    public:
        DDSNode(communication::Settings settings);
        void start() {} //TODO

        void advertise(communication::MessageType type);
        void subscribe(communication::MessageType type);
        void unsubscribe(communication::MessageType type);

        void publishRobotControl(communication::RobotControl control);
        void publishRobotAlarm(communication::RobotAlarm alarm);
        void publishRobotSensor(communication::RobotSensor alarm);

    private:
        Participant mDomainParticipant;
        DDSTopics mTopics;
        DDSPublisher mPublisher;
        DDSSubscriber mSubscriber;
    };
    typedef QSharedPointer<DDSNode> DDSNodeInterfacePtr;
}

#endif //DDSNODE_H
