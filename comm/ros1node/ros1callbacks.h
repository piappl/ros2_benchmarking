#ifndef ROS1CALLBACKS_H
#define ROS1CALLBACKS_H

#include <QVariant>
#include <common/messagetypes.h>
#include "ros1messages.h"

namespace roscommunication
{
    class Ros1Callbacks : public QObject
    {
        Q_OBJECT
    signals:
        void callbackProcessed(communication::MessageType type);

    public:
        template <typename T>
        void callback(const typename T::ConstPtr& msg)
        {
            processCallback(msg);
        }

    private:
        void processCallback(const geometry_msgs::Twist::ConstPtr& msg) { cmdVelCallback(msg); }
        void processCallback(const std_msgs::ByteMultiArray::ConstPtr& msg) { bytesCallback(msg); }
        void processCallback(const piap::RobotControl::ConstPtr& msg) { robotControlCallback(msg); }
        void processCallback(const piap::RobotStatus::ConstPtr& msg) { robotStatusCallback(msg); }

        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
        void robotControlCallback(const piap::RobotControl::ConstPtr& msg);
        void robotStatusCallback(const piap::RobotStatus::ConstPtr& msg);
        void bytesCallback(const std_msgs::ByteMultiArray::ConstPtr& msg);
    };
}

#endif // ROS1CALLBACKS_H
