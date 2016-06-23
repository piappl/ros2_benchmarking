#ifndef DDSSUBSCRIBER_H
#define DDSSUBSCRIBER_H

#include <common/messagetypes.h>
#include <common/qosprofiles.h>
#include "ddsinclude.h"
#include "ddstopics.h"

namespace communication
{
    template <typename M>
    class ReaderListener : public Reader<M>::Listener
    {
    public:
        ReaderListener() = delete;
        ReaderListener(std::string name, std::size_t size) : mName(name), mSize(size) { }
        void on_requested_deadline_missed(Reader<M>& dr, const dds::core::status::RequestedDeadlineMissedStatus &status);
        void on_requested_incompatible_qos(Reader<M>& dr, const dds::core::status::RequestedIncompatibleQosStatus &status);
        void on_sample_rejected(Reader<M>& dr, const dds::core::status::SampleRejectedStatus& status);
        void on_data_available(Reader<M>& dr);
        void on_liveliness_changed(Reader<M>& dr, const dds::core::status::LivelinessChangedStatus& status);
        void on_subscription_matched(Reader<M>& dr, const dds::core::status::SubscriptionMatchedStatus& status);
        void on_sample_lost(Reader<M>& dr, const dds::core::status::SampleLostStatus& status);
    private:
        std::string mName;
        std::size_t mSize;
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
            DDSTopics mTopics;
            communication::QoSSettings mQos;

            Reader<messages::RobotControl> mRobotControlReader{dds::core::null_type()};
            Reader<messages::RobotSensor> mRobotSensorReader{dds::core::null_type()};
            Reader<messages::RobotAlarm> mRobotAlarmReader{dds::core::null_type()};

            std::unique_ptr<ReaderListener<messages::RobotControl>> mRobotControlListener;
            std::unique_ptr<ReaderListener<messages::RobotAlarm>> mRobotAlarmListener;
            std::unique_ptr<ReaderListener<messages::RobotSensor>> mRobotSensorListener;
    };
}
#endif //DDSSUBSCRIBER_H
