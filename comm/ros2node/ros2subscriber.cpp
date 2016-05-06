#include "ros2subscriber.h"

using namespace roscommunication;
using namespace communication;

Ros2Subscriber::Ros2Subscriber(Node::SharedPtr node) : mNode(node)
{
}

Ros2SubscriptionListenerInterfacePtr Ros2Subscriber::constructListener(CommunicationNotification n)
{
    Ros2SubscriptionListenerInterfacePtr listenerPtr;
    switch (n)
    {
    case ControlBase:
        listenerPtr.reset(new Ros2SubscriptionListener<geometry_msgs::msg::Twist>(n, mNode));
        break;
    case ControlRobot:
        listenerPtr.reset(new Ros2SubscriptionListener<robot_information_msgs::msg::RobotControl>(n, mNode));
        break;
    case StatusBeacon:
        listenerPtr.reset(new Ros2SubscriptionListener<robot_information_msgs::msg::RobotInformation>(n, mNode));
        break;
    default:
        debug(LOG_ERROR, "Ros2Subscriber", "Invalid type");
        break;
    }
    return listenerPtr;
}

void Ros2Subscriber::subscribe(communication::CommunicationNotification n, bool subscribe)
{
    if (!mNode)
        return;

    if (!mListeners.contains(n))
    {
        Ros2SubscriptionListenerInterfacePtr construct = constructListener(n);
        mListeners.insert(n, construct);
    }
    Ros2SubscriptionListenerInterfacePtr listenerPtr = mListeners.value(n);

    if (subscribe && !listenerPtr->isRunning())
    {
        listenerPtr->start();
        connect(listenerPtr.data(), SIGNAL(messageReceived(communication::CommunicationNotification, QVariant)),
                this, SIGNAL(messageReceived(communication::CommunicationNotification, QVariant)));
    }
    bool shouldStop = !subscribe;
    listenerPtr->setStop(shouldStop);
}
