#include <common/logging.h>
#include <common/communicationutils.h>
#include "ros1callbacks.h"

using namespace roscommunication;
using namespace communication;

void Ros1Callbacks::robotSensorCallback(const ros2eval_msgs::RobotSensor::ConstPtr &msg)
{
    debug(LOG_BENCHMARK, "RECEIVED RobotSensor", "id=%d, size=%lu", msg->id, msg->data.size());
    emit callbackProcessed(MessageTypeRobotSensor);
}

void Ros1Callbacks::robotControlCallback(const ros2eval_msgs::RobotControl::ConstPtr& msg)
{
    debug(LOG_BENCHMARK, "RECEIVED RobotControl", "id=%d, size=%lu, x=%d, y=%d, z=%d", (int)msg->id, sizeof(communication::RobotControl), msg->x, msg->y,  msg->z);
    emit callbackProcessed(MessageTypeRobotControl);
}

void Ros1Callbacks::robotAlarmCallback(const ros2eval_msgs::RobotAlarm::ConstPtr &msg)
{
    debug(LOG_BENCHMARK, "RECEIVED RobotAlarm", "id=%u, size=%lu", msg->id, sizeof(communication::RobotAlarm));
    emit callbackProcessed(MessageTypeRobotAlarm);
}
