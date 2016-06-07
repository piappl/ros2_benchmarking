#include <common/logging.h>
#include <common/communicationutils.h>
#include "ros1callbacks.h"

using namespace roscommunication;
using namespace communication;

void Ros1Callbacks::bytesCallback(const std_msgs::ByteMultiArray::ConstPtr &msg)
{
    //CommunicationUtils::dumpToHex(msg->data.data(), msg->data.size(), "RECEIVED");
    debug(LOG_BENCHMARK, "RECEIVED byte_msg", "id=00, size=%lu", msg->data.size());
    emit callbackProcessed(MessageTypeBytes);
}

void Ros1Callbacks::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    static int recNo = 0;
    recNo++;
    debug(LOG_BENCHMARK, "RECEIVED cmd_vel", "id=%d, x=%lf, turn=%lf, received no %d",
          (int)msg->linear.z, msg->linear.x, msg->angular.z, recNo);

    emit callbackProcessed(MessageTypeCmdVel);
}

void Ros1Callbacks::robotStatusCallback(const piap::RobotStatus::ConstPtr &msg)
{
    debug(LOG_BENCHMARK, "RECEIVED robot_status", "id=%u", msg->emergency_active);
    emit callbackProcessed(MessageTypeRobotStatus);
}

void Ros1Callbacks::robotControlCallback(const piap::RobotControl::ConstPtr &msg)
{
    debug(LOG_BENCHMARK, "RECEIVED robot_control", "id=%u", msg->emergency_active);
    emit callbackProcessed(MessageTypeRobotControl);
}
