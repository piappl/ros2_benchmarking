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
    mPublisher.advertise(type);
}

void DDSNode::subscribe(communication::MessageType type)
{
    mSubscriber.subscribe(type);
}

void DDSNode::unsubscribe(communication::MessageType type)
{
    mSubscriber.unsubscribe(type);
}

void DDSNode::publishRobotControl(communication::RobotControl control)
{
    mPublisher.publishRobotControl(control);
}

void DDSNode::publishRobotAlarm(communication::RobotAlarm alarm)
{
    mPublisher.publishRobotAlarm(alarm);
}

void DDSNode::publishRobotSensor(communication::RobotSensor sensor)
{
    mPublisher.publishRobotSensor(sensor);
}
