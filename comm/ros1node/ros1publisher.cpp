#include <QMap>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#include <common/logging.h>
#include <common/topics.h>
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

    QString fullTopic = Topics::fullTopic(n, "_");
    return fullTopic;
}

void Ros1Publisher::advertise(MessageType n)
{
    switch (n)
    {
        case MessageTypeRobotControl:
            return advertise<ros2eval_msgs::RobotControl>(n);
        case MessageTypeRobotAlarm:
            return advertise<ros2eval_msgs::RobotAlarm>(n);
        case MessageTypeRobotSensor:
            return advertise<ros2eval_msgs::RobotSensor>(n);
        default:
            debug(LOG_ERROR, "Ros1Publisher", "Advertise: type %d not supported (need to implement?)", n);
            break;
    }
}

void Ros1Publisher::publishRobotControl(communication::RobotControl control)
{
    advertise(MessageTypeRobotControl);
    if (!mPublishers.contains(MessageTypeRobotControl) || !isNodeOperating())
    {
        const char* reason = !mPublishers.contains(MessageTypeRobotControl) ? "NOAD" : "NOOP";
        debug(LOG_BENCHMARK, "WONTPUBLISH RobotControl", "id=%d, reason=%s", control.id, reason);
        return;
    }
    ros2eval_msgs::RobotControl c;
    c.id = control.id;
    c.x = control.x;
    c.y = control.y;
    c.z = control.z;
    debug(LOG_BENCHMARK, "PUBLISHING RobotControl", "id=%d, size=%lu, x=%d, y=%d, z=%d", c.id, sizeof(communication::RobotControl), c.x, c.y, c.z);
    mPublishers.value(MessageTypeRobotControl)->publish(c);
}

void Ros1Publisher::publishRobotAlarm(communication::RobotAlarm alarm)
{
    advertise(MessageTypeRobotAlarm);
    if (!mPublishers.contains(MessageTypeRobotAlarm) || !isNodeOperating())
    {
        const char* reason = !mPublishers.contains(MessageTypeRobotAlarm) ? "NOAD" : "NOOP";
        debug(LOG_BENCHMARK, "WONTPUBLISH Robotalarm", "id=%d, reason=%s", alarm.id, reason);
        return;
    }
    ros2eval_msgs::RobotAlarm c;
    c.id = alarm.id;
    c.alarm1 = alarm.alarm1;
    c.alarm2 = alarm.alarm2;
    debug(LOG_BENCHMARK, "PUBLISHING RobotAlarm", "id=%d, size=%lu", c.id, sizeof(communication::RobotAlarm));
    mPublishers.value(MessageTypeRobotAlarm)->publish(c);
}

void Ros1Publisher::publishRobotSensor(communication::RobotSensor sensor)
{
    advertise(MessageTypeRobotSensor);
    if (!mPublishers.contains(MessageTypeRobotSensor) || !isNodeOperating())
    {
        const char* reason = !mPublishers.contains(MessageTypeRobotSensor) ? "NOAD" : "NOOP";
        debug(LOG_BENCHMARK, "WONTPUBLISH RobotSensor", "id=%d, reason=%s", sensor.id, reason);
        return;
    }
    ros2eval_msgs::RobotSensor c;
    c.id = sensor.id;
    c.data = sensor.data;
    debug(LOG_BENCHMARK, "PUBLISHING RobotSensor", "id=%d, size=%lu", c.id, c.data.size());
    mPublishers.value(MessageTypeRobotSensor)->publish(c);
}
