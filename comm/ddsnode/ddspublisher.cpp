#include "ddspublisher.h"
#include <common/communicationutils.h>
#include <common/logging.h>

using namespace ddscommunication;
using namespace communication;

DDSPublisher::DDSPublisher(const Participant& participant, const DDSTopics &topics)
    : mPublisher(participant),
      mCmdVelWriter(mPublisher, topics.topicCmdVel()),
      mBytesWriter(mPublisher, topics.topicBytes()),
      mControlWriter(mPublisher, topics.topicControl()),
      mStatusWriter(mPublisher, topics.topicStatus())
{   //TODO - don't register writers that are not going to publish ?
}

void DDSPublisher::publishCmdVel(communication::MoveBase base)
{
    MoveBaseDDSType vel(base.id, base.x, base.y, base.z);
    debug(LOG_BENCHMARK, "PUBLISHING cmd_vel", "id=%u", base.id);
    mCmdVelWriter.write(vel);
}

void DDSPublisher::publishByteMessage(int size)
{
    int id = size; //TODO
    QString generated = CommunicationUtils::randomString();
    BytesDDSType message(id, generated.toStdString());
    CommunicationUtils::dumpToHex((int8_t*)generated.toLocal8Bit().data(), generated.size(), "PUBLISHING");
    mBytesWriter.write(message);
}

void DDSPublisher::publishRobotControl(RobotControl c)
{
    RobotControlDDSType message(c.id, c.field1, c.field2, c.field3, c.field4, c.field5, c.field6, c.field7);
    debug(LOG_BENCHMARK, "PUBLISHING robot_control", "id=%u", message.id());
    mControlWriter.write(message);
}

void DDSPublisher::publishRobotStatus(RobotStatus c)
{
    RobotStatusDDSType message(c.id, c.field1, c.field2, c.field3, c.field4, c.field5, c.field6, c.field7);
    debug(LOG_BENCHMARK, "PUBLISHING robot_status", "id=%u", message.id());
    mStatusWriter.write(message);
}

void DDSPublisher::advertise(communication::MessageType type)
{
    //TODO - advertise in DDS?
    Q_UNUSED(type);
}
