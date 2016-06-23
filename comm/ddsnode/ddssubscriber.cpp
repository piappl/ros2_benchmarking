#include "ddssubscriber.h"
#include "ddsqos.h"
#include <common/logging.h>

using namespace communication;

template <typename M>
void ReaderListener<M>::on_requested_deadline_missed(Reader<M>& /*dr*/, const dds::core::status::RequestedDeadlineMissedStatus &status)
{
    debug(LOG_BENCHMARK, mName.data(), "Called Reader::on_requested_deadline_missed (total_count: %d)", status.total_count());
}

template <typename M>
void ReaderListener<M>::on_requested_incompatible_qos(Reader<M>& /*dr*/, const dds::core::status::RequestedIncompatibleQosStatus &status)
{
    debug(LOG_BENCHMARK, mName.data(), "Called Reader::on_requested_incompatible_qos (total_count: %d)", status.total_count());
}

template <typename M>
void ReaderListener<M>::on_sample_rejected(Reader<M>& /*dr*/, const dds::core::status::SampleRejectedStatus& status)
{
    debug(LOG_BENCHMARK, mName.data(), "Called Reader::on_sample_rejected (total_count: %d)", status.total_count());
}

template <typename M>
void ReaderListener<M>::on_data_available(Reader<M>& dr)
{
    auto samples =  dr.take();
    for (auto& s : samples)
    {
        if (s.info().valid())
        {
            debug(LOG_BENCHMARK, ("RECEIVED " + mName).data(), "id=%d, size=%lu", s.data().id(), mSize);
        }
    }
}

template <typename M>
void ReaderListener<M>::on_liveliness_changed(Reader<M>& /*dr*/, const dds::core::status::LivelinessChangedStatus& status)
{
    debug(LOG_BENCHMARK, mName.data(), "Called Reader::on_liveliness_changed (alive: %d, not_alive: %d)", status.alive_count(), status.not_alive_count());
}

template <typename M>
void ReaderListener<M>::on_subscription_matched(Reader<M>& /*dr*/, const dds::core::status::SubscriptionMatchedStatus& status)
{
    debug(LOG_BENCHMARK, mName.data(), "Called Reader::on_subscription_matched (current_count: %d)", status.current_count());
}

template <typename M>
void ReaderListener<M>::on_sample_lost(Reader<M>& /*dr*/, const dds::core::status::SampleLostStatus& status)
{
    debug(LOG_BENCHMARK, mName.data(), "Called Reader::on_sample_lost (total_count: %d)", status.total_count());
}

DDSSubscriber::DDSSubscriber(const Participant &participant, const DDSTopics &topics, communication::QoSSettings qos)
    : mSubsciber(participant),
      mTopics(topics),
      mQos(qos)
{
}

void DDSSubscriber::subscribe(communication::MessageType t)
{
    dds::core::status::StatusMask mask = dds::core::status::StatusMask::all();

    switch (t)
    {
    case communication::MessageTypeRobotControl:
    {
        debug(LOG_BENCHMARK, "DDSSubscriber", "Subscribing to RobotControl messages");
        mRobotControlListener.reset(new ReaderListener<messages::RobotControl>("RobotControl", sizeof(communication::RobotControl)));
        mRobotControlReader = Reader<messages::RobotControl>(mSubsciber, mTopics.topicRobotControl(), getReaderQoS(mQos.value(MessageTypeRobotControl)), mRobotControlListener.get(), mask);
        break;
    }
    case communication::MessageTypeRobotAlarm:
    {
        debug(LOG_BENCHMARK, "DDSSubscriber", "Subscribing to RobotAlarm messages");
        mRobotAlarmListener.reset(new ReaderListener<messages::RobotAlarm>("RobotAlarm", sizeof(communication::RobotAlarm)));
        mRobotAlarmReader = Reader<messages::RobotAlarm>(mSubsciber, mTopics.topicRobotAlarm(), getReaderQoS(mQos.value(MessageTypeRobotAlarm)), mRobotAlarmListener.get(), mask);
        break;    }
    case communication::MessageTypeRobotSensor:
    {
        //TODO: Get string size
        debug(LOG_BENCHMARK, "DDSSubscriber", "Subscribing to RobotSensor messages");
        mRobotSensorListener.reset(new ReaderListener<messages::RobotSensor>("RobotSensor", 200));
        mRobotSensorReader = Reader<messages::RobotSensor>(mSubsciber, mTopics.topicRobotSensor(), getReaderQoS(mQos.value(MessageTypeRobotSensor)), mRobotSensorListener.get(), mask);
        break;
    }
    default:
        break;
    }
}

void DDSSubscriber::unsubscribe(communication::MessageType t)
{
    debug(LOG_WARNING, "DDSSubscriber", "unsubscribe %d", t);
    dds::core::status::StatusMask mask = dds::core::status::StatusMask::none();

    switch (t)
    {
    case communication::MessageTypeRobotControl:
        mRobotControlReader.listener(nullptr, mask);
        break;
    case communication::MessageTypeRobotAlarm:
        mRobotAlarmReader.listener(nullptr, mask);
        break;
    case communication::MessageTypeRobotSensor:
        mRobotSensorReader.listener(nullptr, mask);
        break;
    default:
        break;
    }
}
