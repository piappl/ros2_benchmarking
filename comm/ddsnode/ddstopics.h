#ifndef DDSTOPICS_H
#define DDSTOPICS_H

#include "ddsinclude.h"

namespace ddscommunication
{
    class DDSTopics
    {
    public:
        DDSTopics(const Participant& participant);

        const Topic<MoveBaseDDSType> &topicCmdVel() const { return mCmdVelTopic; }
        const Topic<BytesDDSType> &topicBytes() const { return mBytesTopic; }
        const Topic<RobotControlDDSType> &topicControl() const { return mControlTopic; }
        const Topic<RobotStatusDDSType> &topicStatus() const { return mStatusTopic; }

    private:
        Topic<MoveBaseDDSType> mCmdVelTopic;
        Topic<BytesDDSType> mBytesTopic;
        Topic<RobotControlDDSType> mControlTopic;
        Topic<RobotStatusDDSType> mStatusTopic;
    };
}

#endif //DDSTOPICS_H
