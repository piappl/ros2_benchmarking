#include "ddssubscriber.h"
#include <common/logging.h>

using namespace ddscommunication;

//TODO - remove repetitiveness!

void CmdVelListener::on_data_available(dds::sub::DataReader<MoveBaseDDSType>& dr)
{
    std::cout << "----------cmd_vel on_data_available-----------" << std::endl;
    auto samples =  dr.read();
    std::for_each(samples.begin(),
          samples.end(),
          [](const dds::sub::Sample<MoveBaseDDSType>& s) {
            std::cout << s.data().id() << std::endl;
          });
}

void CmdVelListener::on_liveliness_changed(dds::sub::DataReader<MoveBaseDDSType>& dr,
            const dds::core::status::LivelinessChangedStatus& status)
{
    std::cout << "!!! Liveliness Changed !!!" << std::endl;
}

void RobotControlListener::on_data_available(dds::sub::DataReader<RobotControlDDSType>& dr)
{
    std::cout << "----------robot_control on_data_available-----------" << std::endl;
    auto samples =  dr.read();
    std::for_each(samples.begin(),
          samples.end(),
          [](const dds::sub::Sample<RobotControlDDSType>& s) {
            std::cout << s.data().id() << std::endl;
          });
}

void RobotControlListener::on_liveliness_changed(dds::sub::DataReader<RobotControlDDSType>& dr,
            const dds::core::status::LivelinessChangedStatus& status)
{
    std::cout << "!!! Liveliness Changed !!!" << std::endl;
}

void RobotStatusListener::on_data_available(dds::sub::DataReader<RobotStatusDDSType>& dr)
{
    std::cout << "----------robot_status on_data_available-----------" << std::endl;
    auto samples =  dr.read();
    std::for_each(samples.begin(),
          samples.end(),
          [](const dds::sub::Sample<RobotStatusDDSType>& s) {
            std::cout << s.data().id() << std::endl;
          });
}

void RobotStatusListener::on_liveliness_changed(dds::sub::DataReader<RobotStatusDDSType>& dr,
            const dds::core::status::LivelinessChangedStatus& status)
{
    std::cout << "!!! Liveliness Changed !!!" << std::endl;
}

void BytesListener::on_data_available(dds::sub::DataReader<BytesDDSType>& dr)
{
    std::cout << "----------bytes on_data_available-----------" << std::endl;
    auto samples =  dr.read();
    std::for_each(samples.begin(),
          samples.end(),
          [](const dds::sub::Sample<BytesDDSType>& s) {
            std::cout << s.data().id() << std::endl;
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
      mCmdVelReader(mSubsciber, topics.topicCmdVel()),
      mBytesReader(mSubsciber, topics.topicBytes()),
      mControlReader(mSubsciber, topics.topicControl()),
      mStatusReader(mSubsciber, topics.topicStatus())
{   //TODO - set reader QoS in init list
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
