#include <common/logging.h>
#include <common/topics.h>

#include "ros1subscriber.h"
#include "ros1node.h"

using namespace communication;
using namespace roscommunication;

Ros1Subscriber::Ros1Subscriber(NodeHandlePtr node) : Ros1Participant(node)
{
    qRegisterMetaType<communication::MessageType>("communication::MessageType");
    connect(&mCallbacks, SIGNAL(callbackProcessed(communication::MessageType)),
            this, SLOT(filterCallbacks(communication::MessageType)));
}

void Ros1Subscriber::subscribe(communication::MessageType n, bool sub)
{
    switch (n)
    {
    case MessageTypeRobotControl:
        subscribeToTopic<ros2eval_msgs::RobotControl>(n, sub);
        break;
    case MessageTypeRobotAlarm:
        subscribeToTopic<ros2eval_msgs::RobotAlarm>(n, sub);
        break;
    case MessageTypeRobotSensor:
        subscribeToTopic<ros2eval_msgs::RobotSensor>(n, sub);
        break;
    default:
        debug(LOG_ERROR, "Ros1Subscriber", "invalid topic to subscribe!");
        break;
    }
}

void Ros1Subscriber::synchronize() //Synchronize subscribers
{
    //All topics planned must be subscribed
    foreach (MessageType topic, mTopicsToSubscribe)
    {
        if (mSubscribers.contains(topic))
            continue; //already synchronized

        subscribe(topic);
    }

    //All topics currently subscribed but shouldn't are removed
    foreach (MessageType topic, mSubscribers.keys())
    {
        if (mTopicsToSubscribe.contains(topic))
            continue; //all correct

        mSubscribers.remove(topic);
    }
}

QString Ros1Subscriber::commonSubscribe(MessageType n, bool sub)
{
     if (processSubscribeRequest(n, sub))
        return QString();

    QString fullTopic = Topics::fullTopic(n, "_");
    debug(LOG_WARNING, "Ros1Subscriber", "subscribing for %s", qPrintable(fullTopic));
    return fullTopic;
}

void Ros1Subscriber::removeSubscription(MessageType n)
{
    if (mTopicsToSubscribe.contains(n))
        mTopicsToSubscribe.remove(n);

    if (mSubscribers.contains(n))
        mSubscribers.remove(n);
}

bool Ros1Subscriber::processSubscribeRequest(MessageType n, bool sub)
{
    if (!sub)
    {
        removeSubscription(n);
        return true;
    }

    mTopicsToSubscribe.insert(n);
    if (!isNodeOperating() || mSubscribers.contains(n))
    {
        return true;
    }
    return false;
}

void Ros1Subscriber::filterCallbacks(MessageType type)
{
    if (isNodeOperating())
    {
        emit messageReceived(type);
    }
}
