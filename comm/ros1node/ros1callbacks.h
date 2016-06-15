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
        void processCallback(const messages::RobotControl::ConstPtr& msg) { robotControlCallback(msg); }
        void processCallback(const messages::RobotSensor::ConstPtr& msg) { robotSensorCallback(msg); }
        void processCallback(const messages::RobotAlarm::ConstPtr& msg) { robotAlarmCallback(msg); }

        void robotControlCallback(const messages::RobotControl::ConstPtr& msg);
        void robotAlarmCallback(const messages::RobotAlarm::ConstPtr& msg);
        void robotSensorCallback(const messages::RobotSensor::ConstPtr& msg);
    };
}

#endif // ROS1CALLBACKS_H
