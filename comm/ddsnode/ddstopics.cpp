#include "ddstopics.h"
#include <common/topics.h>

using namespace ddscommunication;
using namespace communication;

namespace
{
    std::string cmdVelTopic() { return Topics::fullTopic(MessageTypeCmdVel, "_").toStdString(); }
    std::string bytesTopic() { return Topics::fullTopic(MessageTypeBytes, "_").toStdString(); }
    std::string controlTopic() { return Topics::fullTopic(MessageTypeRobotControl, "_").toStdString(); }
    std::string statusTopic() { return Topics::fullTopic(MessageTypeRobotStatus, "_").toStdString(); }
}

DDSTopics::DDSTopics(const Participant &participant)
   : mCmdVelTopic(participant, cmdVelTopic()),
     mBytesTopic(participant, bytesTopic()),
     mControlTopic(participant, controlTopic()),
     mStatusTopic(participant, statusTopic())
{
}
