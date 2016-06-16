#include "ddssubscriber.h"
#include "ddsqos.h"
#include <common/logging.h>

using namespace communication;

//TODO - remove repetitiveness!

void RobotControlListener::on_data_available(dds::sub::DataReader<messages::RobotControl>& dr)
{
    auto samples =  dr.take();
    std::for_each(samples.begin(),
          samples.end(),
          [](const dds::sub::Sample<messages::RobotControl>& s) {
            if (s.info().valid())
            {
              debug(LOG_BENCHMARK, "RECEIVED RobotControl", "id=%d, size=%lu", s.data().id(), sizeof(communication::RobotControl));
            }
          });
}

void RobotControlListener::on_liveliness_changed(dds::sub::DataReader<messages::RobotControl>& dr,
            const dds::core::status::LivelinessChangedStatus& status)
{
    std::cout << "!!! Liveliness Changed !!!" << std::endl;
}

void RobotAlarmListener::on_data_available(dds::sub::DataReader<messages::RobotAlarm>& dr)
{
    auto samples =  dr.take();
    std::for_each(samples.begin(),
          samples.end(),
          [](const dds::sub::Sample<messages::RobotAlarm>& s) {
            if (s.info().valid())
            {
              debug(LOG_BENCHMARK, "RECEIVED RobotAlarm", "id=%d, size=%lu", s.data().id(), sizeof(communication::RobotAlarm));
            }
          });
}

void RobotAlarmListener::on_liveliness_changed(dds::sub::DataReader<messages::RobotAlarm>& dr,
            const dds::core::status::LivelinessChangedStatus& status)
{
    std::cout << "!!! Liveliness Changed !!!" << std::endl;
}

void RobotSensorListener::on_data_available(dds::sub::DataReader<messages::RobotSensor>& dr)
{
    auto samples =  dr.take();
    std::for_each(samples.begin(),
          samples.end(),
          [](const dds::sub::Sample<messages::RobotSensor>& s) {
            if (s.info().valid())
            {
              debug(LOG_BENCHMARK, "RECEIVED RobotSensor", "id=%d, size=%lu",s.data().id(), s.data().data().size());
            }
          });
}

void RobotSensorListener::on_liveliness_changed(dds::sub::DataReader<messages::RobotSensor>& dr,
            const dds::core::status::LivelinessChangedStatus& status)
{
    std::cout << "!!! Liveliness Changed !!!" << std::endl;
}


DDSSubscriber::DDSSubscriber(const Participant &participant,
                             const DDSTopics &topics, communication::QoSSettings qos)
    : mSubsciber(participant),
      mRobotControlReader(mSubsciber, topics.topicRobotControl(), getReaderQoS(qos.value(MessageTypeRobotControl))),
      mRobotSensorReader(mSubsciber, topics.topicRobotSensor(), getReaderQoS(qos.value(MessageTypeRobotSensor))),
      mRobotAlarmReader(mSubsciber, topics.topicRobotAlarm(), getReaderQoS(qos.value(MessageTypeRobotAlarm)))
{
    debug(LOG_WARNING, "DDSSubscriber", "ctor");
}

void DDSSubscriber::subscribe(communication::MessageType t)
{
    debug(LOG_WARNING, "DDSSubscriber", "subscribe %d", t);
    dds::core::status::StatusMask mask = dds::core::status::StatusMask::data_available();

    switch (t)
    {
    case communication::MessageTypeRobotControl:
        mRobotControlReader.listener(&mRobotControlListener, mask);
        break;
    case communication::MessageTypeRobotAlarm:
        mRobotAlarmReader.listener(&mRobotAlarmListener, mask);
        break;
    case communication::MessageTypeRobotSensor:
        mRobotSensorReader.listener(&mRobotSensorListener, mask);
        break;
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
