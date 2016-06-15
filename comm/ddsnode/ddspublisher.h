#ifndef DDSPUBLISHER_H
#define DDSPUBLISHER_H

#include <common/messagetypes.h>
#include <common/qosprofiles.h>
#include "ddsinclude.h"
#include "ddstopics.h"

namespace communication
{
    class DDSPublisher
    {
        public:
            DDSPublisher(const Participant& participant,
                         const DDSTopics &topics, communication::QoSSettings qos);
            void advertise(communication::MessageType type);
            void publishRobotControl(communication::RobotControl control);
            void publishRobotAlarm(communication::RobotAlarm alarm);
            void publishRobotSensor(communication::RobotSensor sensor);

        private:
            Publisher mPublisher;

            //TODO - generalize?
            Writer<messages::RobotControl> mRobotControlWriter;
            Writer<messages::RobotSensor> mRobotSensorWriter;
            Writer<messages::RobotAlarm> mRobotAlarmWriter;
    };
}
#endif //DDSPUBLISHER_H
