#ifndef ROS1PUBLISHER_H
#define ROS1PUBLISHER_H

#include <QMap>
#include <QSet>
#include <ros/ros.h>

#include "ros1participant.h"
#include <common/messagetypes.h>
#include <common/logging.h>

namespace roscommunication
{
    class Ros1Node;
    class Ros1Publisher : public Ros1Participant
    {
    public:
        Ros1Publisher(NodeHandlePtr node);
        void advertise(communication::MessageType type);

        void publishRobotControl(communication::RobotControl control);
        void publishRobotAlarm(communication::RobotAlarm alarm);
        void publishRobotSensor(communication::RobotSensor);

    private:     
        template <typename T>
        void advertise(communication::MessageType type)
        {
            QString fullTopic = commonAdvertise(type);
            if (fullTopic.isEmpty())
                return;

            bool latch = false;
            std::string publisherTopic = fullTopic.toStdString();
            ros::Publisher pub = nodeHandle()->advertise<T>(publisherTopic, kQueueSize, latch);
            debug(LOG_WARNING, "Ros1Publisher", "advertising %s", qPrintable(fullTopic));
            mPublishers.insert(type, PublisherPtr(new ros::Publisher(pub)));
        }

        QString commonAdvertise(communication::MessageType n);
        void synchronize();

        typedef QSharedPointer<ros::Publisher> PublisherPtr;
        QMap<communication::MessageType, PublisherPtr> mPublishers;
        QSet<communication::MessageType> mTopicsToAdvertise;
    };
}

#endif // ROS1PUBLISHER_H
