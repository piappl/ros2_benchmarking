#ifndef ROS1FWD_H
#define ROS1FWD_H

#include <QSharedPointer>

namespace ros
{
    class NodeHandle;
    class AsyncSpinner;
}

namespace roscommunication
{
    typedef QSharedPointer<ros::NodeHandle> NodeHandlePtr;
    typedef QSharedPointer<ros::AsyncSpinner> AsyncSpinnerPtr;

    class Ros1Subscriber;
    typedef QSharedPointer<Ros1Subscriber> Ros1SubscriberPtr;

    class Ros1Publisher;
    typedef QSharedPointer<Ros1Publisher> Ros1PublisherPtr;

    class Ros1Node;
    typedef QSharedPointer<Ros1Node> Ros1NodePtr;
}

#endif // ROS1FWD_H
