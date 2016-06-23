#ifndef DDSTOPICS_H
#define DDSTOPICS_H

#include "ddsinclude.h"
#include <common/qosprofiles.h>

namespace communication
{
    class DDSTopics
    {
    public:
        DDSTopics(const Participant& participant, communication::QoSSettings qos);

        const Topic<messages::RobotControl> &topicRobotControl() const { return mRobotControlTopic; }
        const Topic<messages::RobotSensor> &topicRobotSensor() const { return mRobotSensorTopic; }
        const Topic<messages::RobotAlarm> &topicRobotAlarm() const { return mRobotAlarmTopic; }

    private:
        Topic<messages::RobotControl> mRobotControlTopic;
        Topic<messages::RobotSensor> mRobotSensorTopic;
        Topic<messages::RobotAlarm> mRobotAlarmTopic;
    };
}

#endif //DDSTOPICS_H
