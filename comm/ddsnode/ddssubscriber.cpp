#include "ddssubscriber.h"
#include "ddsqos.h"
#include <common/logging.h>

using namespace ddscommunication;
using namespace communication;

//TODO - remove repetitiveness!

void CmdVelListener::on_data_available(dds::sub::DataReader<MoveBaseDDSType>& dr)
{
    auto samples =  dr.take();
    std::for_each(samples.begin(),
          samples.end(),
          [](const dds::sub::Sample<MoveBaseDDSType>& s) {
            debug(LOG_BENCHMARK, "RECEIVED cmd_vel", "id=%d, x=%d, y=%d", s.data().id(), s.data().x(), s.data().y());
          });
}

void CmdVelListener::on_liveliness_changed(dds::sub::DataReader<MoveBaseDDSType>& dr,
            const dds::core::status::LivelinessChangedStatus& status)
{
    std::cout << "!!! Liveliness Changed !!!" << std::endl;
}

void RobotControlListener::on_data_available(dds::sub::DataReader<RobotControlDDSType>& dr)
{
    auto samples =  dr.take();
    std::for_each(samples.begin(),
          samples.end(),
          [](const dds::sub::Sample<RobotControlDDSType>& s) {
            debug(LOG_BENCHMARK, "RECEIVED robot_control", "id=%d", s.data().id());
          });
}

void RobotControlListener::on_liveliness_changed(dds::sub::DataReader<RobotControlDDSType>& dr,
            const dds::core::status::LivelinessChangedStatus& status)
{
    std::cout << "!!! Liveliness Changed !!!" << std::endl;
}

void RobotStatusListener::on_data_available(dds::sub::DataReader<RobotStatusDDSType>& dr)
{
    auto samples =  dr.take();
    std::for_each(samples.begin(),
          samples.end(),
          [](const dds::sub::Sample<RobotStatusDDSType>& s) {
            std::cout << s.data().id() << std::endl;
            debug(LOG_BENCHMARK, "RECEIVED robot_status", "id=%d", s.data().id());
          });
}

void RobotStatusListener::on_liveliness_changed(dds::sub::DataReader<RobotStatusDDSType>& dr,
            const dds::core::status::LivelinessChangedStatus& status)
{
    std::cout << "!!! Liveliness Changed !!!" << std::endl;
}

void BytesListener::on_data_available(dds::sub::DataReader<BytesDDSType>& dr)
{
    auto samples =  dr.take();
    std::for_each(samples.begin(),
          samples.end(),
          [](const dds::sub::Sample<BytesDDSType>& s) {
            debug(LOG_BENCHMARK, "RECEIVED byte_msg", "id=%d", s.data().id());
          });
}

void BytesListener::on_liveliness_changed(dds::sub::DataReader<BytesDDSType>& dr,
            const dds::core::status::LivelinessChangedStatus& status)
{
    std::cout << "!!! Liveliness Changed !!!" << std::endl;
}


DDSSubscriber::DDSSubscriber(const Participant &participant,
                             const DDSTopics &topics, communication::QoSSettings qos)
    : mSubsciber(participant),
      mCmdVelReader(mSubsciber, topics.topicCmdVel(), getReaderQoS(qos.value(MessageTypeCmdVel))),
      mBytesReader(mSubsciber, topics.topicBytes(), getReaderQoS(qos.value(MessageTypeBytes))),
      mControlReader(mSubsciber, topics.topicControl(), getReaderQoS(qos.value(MessageTypeRobotControl))),
      mStatusReader(mSubsciber, topics.topicStatus(), getReaderQoS(qos.value(MessageTypeRobotStatus)))
{
    debug(LOG_WARNING, "DDSSubscriber", "ctor");
}

void DDSSubscriber::subscribe(communication::MessageType t)
{
    debug(LOG_WARNING, "DDSSubscriber", "subscribe %d", t);
    dds::core::status::StatusMask mask = dds::core::status::StatusMask::data_available();

    switch (t)
    {
    case communication::MessageTypeCmdVel:
        mCmdVelReader.listener(&mCmdVelListener, mask);
        break;
    case communication::MessageTypeRobotControl:
        mControlReader.listener(&mControlListener, mask);
        break;
    case communication::MessageTypeRobotStatus:
        mStatusReader.listener(&mStatusListener, mask);
        break;
    case communication::MessageTypeBytes:
        mBytesReader.listener(&mBytesListener, mask);
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
    case communication::MessageTypeCmdVel:
        mCmdVelReader.listener(nullptr, mask);
        break;
    case communication::MessageTypeRobotControl:
        mControlReader.listener(nullptr, mask);
        break;
    case communication::MessageTypeRobotStatus:
        mStatusReader.listener(nullptr, mask);
        break;
    case communication::MessageTypeBytes:
        mBytesReader.listener(nullptr, mask);
        break;
    default:
        break;
    }
}
