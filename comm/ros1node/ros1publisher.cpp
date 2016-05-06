#include <QMap>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#include <common/logging.h>
#include <common/rostopics.h>
#include <common/communicationutils.h>
#include "ros1messages.h"
#include "ros1publisher.h"

using namespace communication;
using namespace roscommunication;

Ros1Publisher::Ros1Publisher(NodeHandlePtr node)
    : Ros1Participant(node)
{
}

void Ros1Publisher::synchronize() //Synchronize advertisers
{
    //All topics planned must be advertised
    foreach (MessageType topic, mTopicsToAdvertise)
    {
        if (mPublishers.contains(topic))
            continue; //already synchronized

        advertise(topic);
    }

    //All topics currently advertised but shouldn't are removed
    foreach (MessageType topic, mPublishers.keys())
    {
        if (mTopicsToAdvertise.contains(topic))
            continue; //all correct

        mPublishers.remove(topic);
    }
}

QString Ros1Publisher::commonAdvertise(MessageType n)
{
    mTopicsToAdvertise.insert(n);
    if (!isNodeOperating() || mPublishers.contains(n))
    {
        return QString();
    }

    QString fullTopic = RosTopics::fullTopic(n);
    return fullTopic;
}

void Ros1Publisher::advertise(MessageType n)
{
    switch (n)
    {
        case MessageTypeCmdVel:
            return advertise<geometry_msgs::Twist>(n);
        case MessageTypeRobotControl:
            return advertise<piap::RobotControl>(n);
        case MessageTypeRobotStatus:
            return advertise<piap::RobotStatus>(n);
        case MessageTypeBytes:
            return advertise<std_msgs::ByteMultiArray>(n);
        default:
            debug(LOG_ERROR, "Ros1Publisher", "Advertise: type %d not supported (need to implement?)", n);
            break;
    }
}

void Ros1Publisher::publishCmdVel(MoveBase mobileBase)
{
    static int noPub = 0;
    static int noFailed = 0;

    advertise(MessageTypeCmdVel);
    if (!mPublishers.contains(MessageTypeCmdVel) || !isNodeOperating())
    {   //Can't publish - no publisher yet
        noFailed++;
        debug(LOG_BENCHMARK, "WONTPUBLISH cmd_vel", "reason: %s, count failed: %d",
              !mPublishers.contains(MessageTypeCmdVel) ? "not advertised yet" : "node not operating",
              noFailed);
        return;
    }
    noPub++;
    geometry_msgs::Twist cmdVel;
    cmdVel.linear.x = mobileBase.x;
    cmdVel.angular.z = mobileBase.z;
    cmdVel.linear.z = mobileBase.id;

    debug(LOG_BENCHMARK, "PUBLISHING cmd_vel", "id=%d, x=%lf, turn=%lf, no pubs %d, no general %d",
          mobileBase.id, cmdVel.linear.x, cmdVel.angular.z, noPub, noFailed+noPub);
    mPublishers.value(MessageTypeCmdVel)->publish(cmdVel);
}

void Ros1Publisher::publishRobotStatus(RobotStatus status)
{
    advertise(MessageTypeRobotStatus);
    if (!mPublishers.contains(MessageTypeRobotStatus) || !isNodeOperating())
    {   //Can't publish - no publisher yet
        return;
    }
    piap::RobotStatus s;
    s.battery = status.field1;
    s.battery_charging = status.field2;
    s.brake_active = status.field3;
    s.emergency_active = status.id; //TODO
    s.drive_reversed = status.field4;
    s.turtle_factor = status.field5;
    debug(LOG_BENCHMARK, "PUBLISHING robot_status", "id=%u", s.emergency_active);
    mPublishers.value(MessageTypeRobotStatus)->publish(s);
}

void Ros1Publisher::publishRobotControl(RobotControl control)
{
    advertise(MessageTypeRobotControl);
    if (!mPublishers.contains(MessageTypeRobotControl) || !isNodeOperating())
    {   //Can't publish - no publisher yet
        return;
    }
    piap::RobotControl c;
    c.emergency_active = control.id;
    c.drive_reversed = control.field1;
    c.turtle = control.field2;
    debug(LOG_BENCHMARK, "PUBLISHING robot_control", "id=%u", c.emergency_active);
    mPublishers.value(MessageTypeRobotControl)->publish(c);
}

void Ros1Publisher::publishByteMessage(int size)
{
    advertise(MessageTypeBytes);
    if (!mPublishers.contains(MessageTypeBytes) || !isNodeOperating())
    {   //Can't publish - no publisher yet
        return;
    }

    if (size < 1)
    {
        debug(LOG_ERROR, "Ros1Node::publishTestMessage", "invalid size");
        return;
    }
    std_msgs::ByteMultiArray msg;
    CommunicationUtils::fillRandomVector(size, msg.data);
    CommunicationUtils::dumpToHex(msg.data.data(), msg.data.size(), "PUBLISHING");
    mPublishers.value(MessageTypeBytes)->publish(msg);
}
