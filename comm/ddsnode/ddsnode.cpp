#include "ddsnode.h"
#include <common/topics.h>

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

void DDSNode::publishRobotControl(communication::RobotControl control)
{
    debug(LOG_WARNING, "DDSNODE", "publishCmdVel");
    mPublisher.publishRobotControl(control);
}

void DDSNode::publishRobotAlarm(communication::RobotAlarm alarm)
{
    debug(LOG_WARNING, "DDSNODE", "publishRobotControl");
    mPublisher.publishRobotAlarm(alarm);
}

void DDSNode::publishRobotSensor(communication::RobotSensor sensor)
{
    debug(LOG_WARNING, "DDSNODE", "publishByteMessage");
    mPublisher.publishRobotSensor(sensor);
}
