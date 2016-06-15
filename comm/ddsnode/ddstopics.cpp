#include "ddstopics.h"
#include <common/topics.h>

using namespace communication;

namespace
{
    std::string robotControlTopic() { return Topics::fullTopic(MessageTypeRobotControl, "_").toStdString(); }
    std::string robotSensorTopic() { return Topics::fullTopic(MessageTypeRobotSensor, "_").toStdString(); }
    std::string robotAlarmTopic() { return Topics::fullTopic(MessageTypeRobotAlarm, "_").toStdString(); }
}

DDSTopics::DDSTopics(const Participant &participant)
   : mRobotControlTopic(participant, robotControlTopic()),
     mRobotSensorTopic(participant, robotSensorTopic()),
     mRobotAlarmTopic(participant, robotAlarmTopic())
{
}
