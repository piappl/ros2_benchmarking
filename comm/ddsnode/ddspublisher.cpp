#include "ddspublisher.h"
#include "ddsqos.h"
#include <common/communicationutils.h>
#include <common/logging.h>

using namespace communication;

template <typename M>
void WriterListener<M>::on_offered_deadline_missed(Writer<M>& /*dw*/, const dds::core::status::OfferedDeadlineMissedStatus& status)
{
    debug(LOG_BENCHMARK, mName.data(), "Called Writer::on_offered_deadline_missed (total_count: %d)", status.total_count());
}

template <typename M>
void WriterListener<M>::on_offered_incompatible_qos(Writer<M>& /*dw*/, const dds::core::status::OfferedIncompatibleQosStatus&  status)
{
    debug(LOG_BENCHMARK, mName.data(), "Called Writer::on_offered_incompatible_qos (total_count: %d)", status.total_count());
}

template <typename M>
void WriterListener<M>::on_liveliness_lost(Writer<M>& /*dw*/, const dds::core::status::LivelinessLostStatus& status)
{
    debug(LOG_BENCHMARK, mName.data(), "Called Writer::on_liveliness_lost (total_count: %d)", status.total_count());
}

template <typename M>
void WriterListener<M>::on_publication_matched(Writer<M>& /*dw*/, const dds::core::status::PublicationMatchedStatus& status)
{
    debug(LOG_BENCHMARK, mName.data(), "Called Writer::on_publication_matched (current_count: %d)", status.current_count());
}

DDSPublisher::DDSPublisher(const Participant& participant, const DDSTopics &topics, communication::QoSSettings qos)
    : mPublisher(participant),
      mTopics(topics),
      mQos(qos)
{
}

void DDSPublisher::publishRobotControl(communication::RobotControl control)
{
    messages::RobotControl message(control.id, control.x, control.y, control.z, 1);
    debug(LOG_BENCHMARK, "PUBLISHING RobotControl", "id=%d, size=%lu", message.id(), sizeof(communication::RobotControl));
    try
    {
        mRobotControlWriter.write(message);
    }
    catch(dds::core::Exception& e)
    {
        debug(LOG_BENCHMARK, "Exception", "mRobotControlWriter.write: %s", e.what());
    }
}

void DDSPublisher::publishRobotSensor(communication::RobotSensor sensor)
{
    messages::RobotSensor message(sensor.id, sensor.data, 2);
    debug(LOG_BENCHMARK, "PUBLISHING RobotSensor", "id=%d, size=%lu", message.id(), message.data().size());
    try
    {
        mRobotSensorWriter.write(message);
    }
    catch(dds::core::Exception& e)
    {
        debug(LOG_BENCHMARK, "Exception", "mRobotSensorWriter.write: %s", e.what());
    }
}

void DDSPublisher::publishRobotAlarm(RobotAlarm alarm)
{
    //static int count = 0;
    messages::RobotAlarm message(alarm.id, alarm.alarm1, alarm.alarm2, 3);
    debug(LOG_BENCHMARK, "PUBLISHING RobotAlarm", "id=%d, size=%lu", message.id(), sizeof(communication::RobotAlarm));
    try
    {
        mRobotAlarmWriter.write(message);
    }
    catch(dds::core::Exception& e)
    {
        debug(LOG_BENCHMARK, "Exception", "mRobotAlarmWriter.write: %s", e.what());
    }
}

void DDSPublisher::advertise(communication::MessageType t)
{
    dds::core::status::StatusMask mask = dds::core::status::StatusMask::all();

    switch (t)
    {
    case communication::MessageTypeRobotControl:
        debug(LOG_BENCHMARK, "DDSSubscriber", "Advertising RobotControl messages");
        mRobotControlListener.reset(new WriterListener<messages::RobotControl>("RobotControl", sizeof(communication::RobotControl)));
        mRobotControlWriter = Writer<messages::RobotControl>(mPublisher, mTopics.topicRobotControl(), getWriterQoS(mQos.value(MessageTypeRobotControl)), mRobotControlListener.get(), mask);
        break;
    case communication::MessageTypeRobotAlarm:
        debug(LOG_BENCHMARK, "DDSSubscriber", "Advertising RobotAlarm messages");
        mRobotAlarmListener.reset(new WriterListener<messages::RobotAlarm>("RobotAlarm", sizeof(communication::RobotAlarm)));
        mRobotAlarmWriter = Writer<messages::RobotAlarm>(mPublisher, mTopics.topicRobotAlarm(), getWriterQoS(mQos.value(MessageTypeRobotAlarm)), mRobotAlarmListener.get(), mask);
        break;
    case communication::MessageTypeRobotSensor:
        debug(LOG_BENCHMARK, "DDSSubscriber", "Advertising RobotSensor messages");
        mRobotSensorListener.reset(new WriterListener<messages::RobotSensor>("RobotSensor", 200));
        mRobotSensorWriter = Writer<messages::RobotSensor>(mPublisher, mTopics.topicRobotSensor(), getWriterQoS(mQos.value(MessageTypeRobotSensor)), mRobotSensorListener.get(), mask);
        break;
    default:
        break;
    }
}
