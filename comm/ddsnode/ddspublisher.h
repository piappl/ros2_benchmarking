#ifndef DDSPUBLISHER_H
#define DDSPUBLISHER_H

#include <common/messagetypes.h>
#include <common/qosprofiles.h>
#include "ddsinclude.h"
#include "ddstopics.h"

namespace communication
{
    template <typename M>
    class WriterListener : public Writer<M>::Listener
    {
    public:
        WriterListener() = delete;
        WriterListener(std::string name, std::size_t size) : mName(name), mSize(size) { }
        void on_offered_deadline_missed(Writer<M>& writer, const dds::core::status::OfferedDeadlineMissedStatus& status);
        void on_offered_incompatible_qos(Writer<M>& writer, const dds::core::status::OfferedIncompatibleQosStatus&  status);
        void on_liveliness_lost(Writer<M>& writer, const dds::core::status::LivelinessLostStatus& status);
        void on_publication_matched(Writer<M>& writer, const dds::core::status::PublicationMatchedStatus& status);
    private:
        std::string mName;
        std::size_t mSize;
    };

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
            DDSTopics mTopics;
            communication::QoSSettings mQos;

            Writer<messages::RobotControl> mRobotControlWriter{dds::core::null_type()};
            Writer<messages::RobotSensor> mRobotSensorWriter{dds::core::null_type()};
            Writer<messages::RobotAlarm> mRobotAlarmWriter{dds::core::null_type()};

            std::unique_ptr<WriterListener<messages::RobotControl>> mRobotControlListener;
            std::unique_ptr<WriterListener<messages::RobotAlarm>> mRobotAlarmListener;
            std::unique_ptr<WriterListener<messages::RobotSensor>> mRobotSensorListener;
    };
}
#endif //DDSPUBLISHER_H
