#ifndef ROS2SUBSCRIBER_H
#define ROS2SUBSCRIBER_H

#include <QMap>
#include <communication/communicationevents.h>
#include "ros2subscriptionlistener.h"

namespace roscommunication
{
    class Ros2Subscriber : public QObject
    {
        Q_OBJECT
    signals:
        void messageReceived(communication::CommunicationNotification n, QVariant content);

    public:
        Ros2Subscriber(Node::SharedPtr node);
        void subscribe(communication::CommunicationNotification n, bool subscribe = true);

    private:
        typedef QMap<communication::CommunicationNotification, Ros2SubscriptionListenerInterfacePtr> Listeners;

        Ros2SubscriptionListenerInterfacePtr constructListener(communication::CommunicationNotification n);
        Listeners mListeners;
        Node::SharedPtr mNode;
    };
    typedef QSharedPointer<Ros2Subscriber> Ros2SubscriberPtr;
}

#endif // ROS2SUBSCRIBER_H
