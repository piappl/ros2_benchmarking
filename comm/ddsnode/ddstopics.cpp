#include "ddstopics.h"
#include "ddsqos.h"
#include <common/topics.h>

using namespace communication;

namespace
{
    std::string robotControlTopic() { return Topics::fullTopic(MessageTypeRobotControl, "_").toStdString(); }
    std::string robotSensorTopic() { return Topics::fullTopic(MessageTypeRobotSensor, "_").toStdString(); }
    std::string robotAlarmTopic() { return Topics::fullTopic(MessageTypeRobotAlarm, "_").toStdString(); }
}

DDSTopics::DDSTopics(const Participant &participant, communication::QoSSettings qos)
   : mRobotControlTopic(participant, robotControlTopic(), getTopicQoS(qos.value(MessageTypeRobotControl))),
     mRobotSensorTopic(participant, robotSensorTopic(), getTopicQoS(qos.value(MessageTypeRobotSensor))),
     mRobotAlarmTopic(participant, robotAlarmTopic(), getTopicQoS(qos.value(MessageTypeRobotAlarm)))
{
}
