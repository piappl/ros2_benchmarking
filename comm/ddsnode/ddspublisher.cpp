#include "ddspublisher.h"
#include "ddsqos.h"
#include <common/communicationutils.h>
#include <common/logging.h>

using namespace communication;

DDSPublisher::DDSPublisher(const Participant& participant,
                           const DDSTopics &topics, communication::QoSSettings qos)
    : mPublisher(participant),
      mRobotControlWriter(mPublisher, topics.topicRobotControl(), getWriterQoS(qos.value(MessageTypeRobotControl))),
      mRobotSensorWriter(mPublisher, topics.topicRobotSensor(), getWriterQoS(qos.value(MessageTypeRobotSensor))),
      mRobotAlarmWriter(mPublisher, topics.topicRobotAlarm(), getWriterQoS(qos.value(MessageTypeRobotAlarm)))
{   //TODO - don't register writers that are not going to publish ?
}

void DDSPublisher::publishRobotControl(communication::RobotControl control)
{
    messages::RobotControl message(control.id, control.x, control.y, control.z);
    debug(LOG_BENCHMARK, "PUBLISHING RobotControl", "id=%d, size=%lu", message.id(), sizeof(communication::RobotControl));
    mRobotControlWriter.write(message);
}

void DDSPublisher::publishRobotSensor(communication::RobotSensor sensor)
{
    messages::RobotSensor message(sensor.id, sensor.data);
    debug(LOG_BENCHMARK, "PUBLISHING RobotSensor", "id=%d, size=%lu", message.id(), message.data().size());
    mRobotSensorWriter.write(message);
}

void DDSPublisher::publishRobotAlarm(RobotAlarm alarm)
{
    messages::RobotAlarm message(alarm.id, alarm.alarm1, alarm.alarm2);
    debug(LOG_BENCHMARK, "PUBLISHING RobotAlarm", "id=%d, size=%lu", message.id(), sizeof(communication::RobotAlarm));
    mRobotAlarmWriter.write(message);
}

void DDSPublisher::advertise(communication::MessageType type)
{
    //TODO - advertise in DDS?
    Q_UNUSED(type);
}
