#ifndef ROS1SUBSCRIBER_H
#define ROS1SUBSCRIBER_H

#include <QSet>
#include <ros/ros.h>
#include "ros1participant.h"
#include "ros1callbacks.h"

namespace roscommunication
{
    class Ros1Subscriber : public QObject, public Ros1Participant
    {
        Q_OBJECT
    signals:
        void messageReceived(communication::MessageType type);

    public:
        Ros1Subscriber(NodeHandlePtr node);
        void subscribe(communication::MessageType n, bool sub = true);

    private slots:
        void filterCallbacks(communication::MessageType type);

    private:
        template <typename T>
        void subscribeToTopic(communication::MessageType n, bool sub = true)
        {
            QString fullTopic = commonSubscribe(n, sub);
            if (fullTopic.isEmpty())
                return;

            std::string subscriberTopic = fullTopic.toStdString();
            ros::Subscriber subs = nodeHandle()->subscribe<T>(subscriberTopic, kQueueSize,
                                                              &Ros1Callbacks::callback<T>, &mCallbacks);

            mSubscribers.insert(n, SubscriberPtr(new ros::Subscriber(subs)));
        }

        QString commonSubscribe(communication::MessageType n, bool sub);
        void removeSubscription(communication::MessageType n);
        bool processSubscribeRequest(communication::MessageType n, bool sub);
        void synchronize();

        typedef QSharedPointer<ros::Subscriber> SubscriberPtr;
        QMap<communication::MessageType, SubscriberPtr> mSubscribers;
        QSet<communication::MessageType> mTopicsToSubscribe;
        Ros1Callbacks mCallbacks;
    };
}

#endif //ROS1SUBSCRIBER_H
