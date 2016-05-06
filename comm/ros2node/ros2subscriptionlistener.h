#ifndef ROS2SUBSCRIPTIONLISTENER_H
#define ROS2SUBSCRIPTIONLISTENER_H

#include "ros2subscriptionlistenerinterface.h"
#include "ros2messagetovariant.h"
#include "ros2messages.h"
#include <roscommunication/rostopics.h>

namespace roscommunication
{
    template <typename T>
    class Ros2SubscriptionListener : public Ros2SubscriptionListenerInterface
    {
    public:
        Ros2SubscriptionListener(communication::CommunicationNotification type, Node::SharedPtr node)
            : Ros2SubscriptionListenerInterface(type), mNode(node)
        {

        }

    private:
        Node::SharedPtr mNode;
        typename rclcpp::subscription::Subscription<T>::SharedPtr mSubscription;

        void subscribe()
        {
            auto callback =
              [this](const typename T::SharedPtr msg) -> void
              {
                this->processMessage(msg);
              };

            QString fullTopicName = RosTopics::fullTopic(messageType(), topicPrefix());
            mSubscription = mNode->create_subscription<T>
                    (fullTopicName.toStdString(), callback, rmw_qos_profile_default);
        }

        void processMessage(const typename T::SharedPtr msg)
        {
           if (isStopped())
           {   //TODO - upgrade. For now, just don't process
               return;
           }

           QVariant content = ros2MessageToVariant(msg);
           emit messageReceived(messageType(), content);
        }

        void spin()
        {
            rclcpp::spin(mNode);
        }
    };
}

#endif // ROS2SUBSCRIPTIONLISTENER_H
