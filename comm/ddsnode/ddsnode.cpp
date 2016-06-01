#include "ddsnode.h"
#include <common/topics.h>

using namespace ddscommunication;
using namespace communication;

DDSNode::DDSNode(Settings settings)
    : mDomainParticipant(settings.domainID),
      mTopics(mDomainParticipant),
      mPublisher(mDomainParticipant, mTopics, settings.qos),
      mSubscriber(mDomainParticipant, mTopics, settings.qos)
{
}

void DDSNode::advertise(communication::MessageType type)
{
    debug(LOG_WARNING, "DDSNODE", "advertise %d", type);
    mPublisher.advertise(type);
}

void DDSNode::subscribe(communication::MessageType type)
{
    debug(LOG_WARNING, "DDSNODE", "subscribe %d", type);
    mSubscriber.subscribe(type);
    debug(LOG_WARNING, "DDSNODE", "subscribed %d", type);
}

void DDSNode::unsubscribe(communication::MessageType type)
{
    debug(LOG_WARNING, "DDSNODE", "unsubscribe %d", type);
    mSubscriber.unsubscribe(type);
    debug(LOG_WARNING, "DDSNODE", "unsubscribed %d", type);
}

void DDSNode::publishCmdVel(communication::MoveBase cmdVel)
{
    debug(LOG_WARNING, "DDSNODE", "publishCmdVel");
    mPublisher.publishCmdVel(cmdVel);
}

void DDSNode::publishRobotStatus(communication::RobotStatus status)
{
    debug(LOG_WARNING, "DDSNODE", "publishRobotStatus");
    mPublisher.publishRobotStatus(status);
}

void DDSNode::publishRobotControl(communication::RobotControl control)
{
    debug(LOG_WARNING, "DDSNODE", "publishRobotControl");
    mPublisher.publishRobotControl(control);
}

void DDSNode::publishByteMessage(int size)
{
    debug(LOG_WARNING, "DDSNODE", "publishByteMessage");
    mPublisher.publishByteMessage(size);
}
