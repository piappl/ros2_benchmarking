#ifndef DDSSUBSCRIBER_H
#define DDSSUBSCRIBER_H

#include <common/messagetypes.h>
#include <common/qosprofiles.h>
#include "ddsinclude.h"
#include "ddstopics.h"

namespace communication
{   //TODO - refactor
    class RobotControlListener : public dds::sub::NoOpDataReaderListener<messages::RobotControl>
    {
    public:
        virtual void on_data_available(dds::sub::DataReader<messages::RobotControl>& dr);
        virtual void on_liveliness_changed(dds::sub::DataReader<messages::RobotControl>& dr,
                  const dds::core::status::LivelinessChangedStatus& status);
    };

    class RobotAlarmListener : public dds::sub::NoOpDataReaderListener<messages::RobotAlarm>
    {
    public:
        virtual void on_data_available(dds::sub::DataReader<messages::RobotAlarm>& dr);
        virtual void on_liveliness_changed(dds::sub::DataReader<messages::RobotAlarm>& dr,
                  const dds::core::status::LivelinessChangedStatus& status);
    };

    class RobotSensorListener : public dds::sub::NoOpDataReaderListener<messages::RobotSensor>
    {
    public:
        virtual void on_data_available(dds::sub::DataReader<messages::RobotSensor>& dr);
        virtual void on_liveliness_changed(dds::sub::DataReader<messages::RobotSensor>& dr,
                  const dds::core::status::LivelinessChangedStatus& status);
    };


    class DDSSubscriber
    {
        public:
            DDSSubscriber(const Participant& participant,
                          const DDSTopics &topics, communication::QoSSettings qos);
            void subscribe(communication::MessageType t);
            void unsubscribe(communication::MessageType t);
        private:
            Subscriber mSubsciber;

            //TODO - generalize (?)
            Reader<messages::RobotControl> mRobotControlReader;
            Reader<messages::RobotSensor> mRobotSensorReader;
            Reader<messages::RobotAlarm> mRobotAlarmReader;

            RobotControlListener mRobotControlListener;
            RobotAlarmListener mRobotAlarmListener;
            RobotSensorListener mRobotSensorListener;
    };
}
#endif //DDSSUBSCRIBER_H
