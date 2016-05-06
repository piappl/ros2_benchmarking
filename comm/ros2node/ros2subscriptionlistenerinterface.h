#ifndef ROS2SUBSCRIPTIONLISTENERINTERFACE_H
#define ROS2SUBSCRIPTIONLISTENERINTERFACE_H

#include <QThread>
#include <QString>
#include <QVariant>
#include <QSharedPointer>

#include <common/messagetypes.h>

namespace roscommunication
{
    class Ros2SubscriptionListenerInterface : public QObject
    {
        Q_OBJECT
    signals:
        void messageReceived(communication::MessageType messageType, QVariant content);

    public:
        Ros2SubscriptionListenerInterface(communication::MessageType messageType)
            : mStopped(false), mMessageType(messageType)
        {
        }

        virtual ~Ros2SubscriptionListenerInterface() {}
        void setStop(bool stop)
        {
            mStopped = stop;
        }

        bool isStopped()
        {
            return mStopped;
        }

    protected:
        communication::MessageType messageType() const { return mMessageType; }

    private:
        bool mStopped;
        communication::MessageType mMessageType;
    };
    typedef QSharedPointer<Ros2SubscriptionListenerInterface> Ros2SubscriptionListenerInterfacePtr;
}

#endif // ROS2SUBSCRIPTIONLISTENERINTERFACE_H
