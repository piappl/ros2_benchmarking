#include <QTimer>
#include <QSet>
#include <QThread>
#include <QVariant>
#include <QSharedPointer>

//TODO - this is an implementation file that can be included only once per executable
//- until alpha3 planned refactor of ROS2 rclcpp into a lib. Until then, the code below is big
// and not partitioned
#include <rclcpp/rclcpp.hpp>

#include <common/logging.h>
#include <common/messagetypes.h>
#include <common/communicationutils.h>
#include <common/rostopics.h>
#include "ros2node.h"
#include "ros2messages.h"
#include "ros2messagetovariant.h"
#include "ros2subscriptionlistenerinterface.h"
#include "ros2qosprofile.h"

//TODO - redesign subscriber

using namespace communication;

namespace roscommunication
{
    class RosInitializer
    {   //Responsible for ROS init, construct before using ROS
        public:
            bool initialized() const { return mInitialized; }

            RosInitializer(QString name) : mInitialized(false)
            {
                char *argv[] = { name.toLocal8Bit().data() };
                int argc = sizeof(argv) / sizeof(char*) - 1;
                rclcpp::utilities::init(argc, argv);
                mInitialized = true; //TODO - check
                debug(LOG_WARNING, "RosInitializer", "ros2 initialized");
            }
        private:
            bool mInitialized;
    };


/*--------------------ROS2SubscribtionListener----------------------*/


    template <typename T>
    class Ros2SubscriptionListener : public Ros2SubscriptionListenerInterface
    {
    public:
        Ros2SubscriptionListener(communication::MessageType type, rclcpp::Node::SharedPtr node)
            : Ros2SubscriptionListenerInterface(type), mNode(node)
        {
            subscribe();
        }

        void processMessage(const typename T::SharedPtr msg)
        {
           //debug(LOG_WARNING, "Ros2node", "process message %d starts", messageType());
           if (isStopped())
           {   //TODO - upgrade. For now, just don't process
               return;
           }

           QVariant content = ros2MessageToVariant(msg);
           emit messageReceived(messageType(), content);
        }

    private:
        rclcpp::Node::SharedPtr mNode;
        typename rclcpp::subscription::Subscription<T>::SharedPtr mSubscription;

        void subscribe()
        {
            auto callback =
              [this](const typename T::SharedPtr msg) -> void
              {
                this->processMessage(msg);
              };

            QString fullTopicName = RosTopics::fullTopic(messageType(), "_");

            debug(LOG_WARNING, "Ros2SubscriptionListener", "Subscribe |%s| mNode %lld",
                  qPrintable(fullTopicName), (qint64)mNode.get());

            mSubscription = mNode->create_subscription<T>
                    (fullTopicName.toStdString(), callback, Ros2QoSProfile::getProfile(messageType()));
            //debug(LOG_WARNING, "Ros2SubscriptionListener", "Subscribed");
        }
    };

    class Ros2ListenerThread : public QThread
    {
        Q_OBJECT
    signals:
        void messageReceived(communication::MessageType messageType, QVariant content);

    public:
        Ros2ListenerThread(rclcpp::Node::SharedPtr node)
            : mNode(node)
        {
        }

        void subscribe(communication::MessageType n)
        {
            //debug(LOG_WARNING, "Ros2ListenerThread", "subscribe attempt to %d", n);
            if (!mListeners.contains(n))
            {
                Ros2SubscriptionListenerInterfacePtr construct = constructListener(n);
                mListeners.insert(n, construct);
                //debug(LOG_WARNING, "Ros2ListenerThread", "subscribed to %d", n);
            }
        }

    protected:
        void run()
        {
            //debug(LOG_WARNING, "Ros2ListenerThread", "Spin");
            spin();
            //debug(LOG_WARNING, "Ros2ListenerThread", "Spin PAST");
        }

    private:
        void spin()
        {
            rclcpp::spin(mNode);
        }

        Ros2SubscriptionListenerInterfacePtr constructListener(communication::MessageType n)
        {
            //debug(LOG_WARNING, "Ros2Subscriber", "constructing listener %d", n);

            Ros2SubscriptionListenerInterfacePtr listenerPtr;
            switch (n)
            {
            case MessageTypeCmdVel:
                listenerPtr.reset(new Ros2SubscriptionListener<geometry_msgs::msg::Transform>(n, mNode));
                break;
            case MessageTypeRobotControl:
                listenerPtr.reset(new Ros2SubscriptionListener<robot_information_msgs::msg::RobotControl>(n, mNode));
                break;
            case MessageTypeRobotStatus:
                listenerPtr.reset(new Ros2SubscriptionListener<robot_information_msgs::msg::RobotStatus>(n, mNode));
                break;
            case MessageTypeBytes:
                listenerPtr.reset(new Ros2SubscriptionListener<std_msgs::msg::ByteMultiArray>(n, mNode));
                break;
            default:
                debug(LOG_ERROR, "Ros2Subscriber", "Invalid type");
                break;
            }
            //debug(LOG_WARNING, "Ros2Subscriber", "constructed");
            connect(listenerPtr.data(), SIGNAL(messageReceived(communication::MessageType,QVariant)),
                    this, SIGNAL(messageReceived(communication::MessageType,QVariant)));

            return listenerPtr;
        }

        typedef QMap<communication::MessageType, Ros2SubscriptionListenerInterfacePtr> Listeners;
        Listeners mListeners;
        rclcpp::Node::SharedPtr mNode;
    };

/*--------------------ROS2Subscriber----------------------*/


    class Ros2Subscriber : public QObject
    {   //TODO - prefix
        Q_OBJECT
    signals:
        void messageReceived(communication::MessageType n, QVariant content);

    public:
        Ros2Subscriber(rclcpp::Node::SharedPtr node) : mNode(node), mListener(node)
        {
            connect(&mListener, SIGNAL(messageReceived(communication::MessageType,QVariant)),
                    this, SIGNAL(messageReceived(communication::MessageType,QVariant)));
        }

        ~Ros2Subscriber()
        {
            //TODO - properly terminate the listener thread (set sth. that stops spin)
        }

        void start()
        {
            startListening();
        }

        void subscribe(communication::MessageType n, bool subscribe = true)
        {
            //TODO (if !subscribe)
            if (!subscribe)
            {
                debug(LOG_ERROR, "Ros2Subscriber", "Unsubscribe: not implemented (yet)!");
                return;
            }

            if (!mNode)
            {
                //debug(LOG_WARNING, "Ros2Subscriber", "Adding planned subscription");
                addPlannedSubscribe(n);
                return;
            }
            if (mPlannedSubscriptions.contains(n))
            {
                removePlannedSubscribe(n);
            }

            //debug(LOG_WARNING, "Ros2Node", "Subscribe to %d", n);
            mListener.subscribe(n);
        }

    private:
        void startListener()
        {
            if (!mListener.isRunning())
            {
                mListener.start();
            }
        }

        void addPlannedSubscribe(communication::MessageType n)
        {
           mPlannedSubscriptions.insert(n);
        }

        void removePlannedSubscribe(communication::MessageType n)
        {
            mPlannedSubscriptions.remove(n);
        }

        void startListening()
        {
            if (!mNode)
            {   //TODO
                debug(LOG_WARNING, "Ros2Subscriber", "Unable to start");
                return;
            }

            foreach (communication::MessageType n, mPlannedSubscriptions)
            {
                mListener.subscribe(n);
            }
            startListener();
        }

        QSet<communication::MessageType> mPlannedSubscriptions;
        rclcpp::Node::SharedPtr mNode;
        Ros2ListenerThread mListener;
    };
    typedef QSharedPointer<Ros2Subscriber> Ros2SubscriberPtr;


/*--------------------ROS2Publisher---------------------*/


    class Ros2Publisher
    {
    public:
        Ros2Publisher(rclcpp::Node::SharedPtr node) : mNode(node), mStarted(false) {}

        void advertise(MessageType n)
        {
            switch (n)
            {
                case MessageTypeCmdVel:
                    return advertise<geometry_msgs::msg::Transform>(n);
                case MessageTypeRobotControl:
                    return advertise<robot_information_msgs::msg::RobotControl>(n);
                case MessageTypeRobotStatus:
                    return advertise<robot_information_msgs::msg::RobotStatus>(n);
                case MessageTypeBytes:
                    return advertise<std_msgs::msg::ByteMultiArray>(n);
                default:
                    debug(LOG_ERROR, "Ros2Publisher", "advertise: not implemented topic for %d", n);
                    break;
            }
        }

        void start()
        {
            mStarted = true;
        }

        void publishByteMessage(QVariant content)
        {
            if (!mStarted)
                return;

            //debug(LOG_DEBUG, "Ros2Publisher", "publish testMessage");

            if (!mNode)
            {
                debug(LOG_ERROR, "Ros2Publisher", "ERROR! No node - cannot publish");
                return;
            }
            MessageType n = MessageTypeBytes;
            advertise(n);
            int size = content.value<int>();

            if (size < 1)
            {
                debug(LOG_ERROR, "Ros2Publisher::publishTestMessage", "invalid size %d", size);
                return;
            }
            auto pub = std::static_pointer_cast<rclcpp::publisher::Publisher<std_msgs::msg::ByteMultiArray> >(mPublishers.value(n));
            std_msgs::msg::ByteMultiArray msg;
            CommunicationUtils::fillRandomVector(size, msg.data);
            CommunicationUtils::dumpToHex((const int8_t*)msg.data.data(), msg.data.size(), "PUBLISHING");
            pub->publish(msg);
        }

        void publishCmdVel(QVariant content)
        {
            if (!mStarted)
                return;

            //debug(LOG_DEBUG, "Ros2Publisher", "publish cmdVel");

            if (!mNode)
            {
                debug(LOG_ERROR, "Ros2Publisher", "ERROR! No node - cannot publish");
                return;
            }
            MessageType n = MessageTypeCmdVel;
            advertise(n);
            MoveBase mobileBase = content.value<MoveBase>();

            geometry_msgs::msg::Transform cmdVel;
            cmdVel.translation.x = mobileBase.x;
            cmdVel.rotation.z = mobileBase.z;
            cmdVel.translation.z = mobileBase.id;

            debug(LOG_BENCHMARK, "PUBLISHING cmd_vel", "id=%d, x=%lf, turn=%lf",
                  mobileBase.id, cmdVel.translation.x, cmdVel.rotation.z);

            auto pub = std::static_pointer_cast<rclcpp::publisher::Publisher<geometry_msgs::msg::Transform> >(mPublishers.value(n));
            pub->publish(cmdVel);
        }

        void publishRobotControl(QVariant content)
        {
            if (!mStarted)
                return;

            //debug(LOG_DEBUG, "Ros2Publisher", "publish robotControl");

            if (!mNode)
            {
                debug(LOG_ERROR, "Ros2Publisher", "ERROR! No node - cannot publish");
                return;
            }
            MessageType n = MessageTypeRobotControl;
            advertise(n);
            RobotControl control = content.value<RobotControl>();
            robot_information_msgs::msg::RobotControl c;

            c.emergency_active = control.field2;
            c.drive_reversed = control.field1;
            c.turtle = control.id; //TODO
            auto pub = std::static_pointer_cast<rclcpp::publisher::Publisher<robot_information_msgs::msg::RobotControl> >(mPublishers.value(n));

            debug(LOG_BENCHMARK, "PUBLISHING robot_control", "id=%u", c.turtle);
            pub->publish(c);
        }

        void publishRobotStatus(QVariant content)
        {
            if (!mStarted)
                return;

            //debug(LOG_WARNING, "Ros2Publisher", "publish robot status");

            if (!mNode)
            {
                debug(LOG_ERROR, "Ros2Publisher", "ERROR! No node - cannot publish");
                return;
            }
            MessageType n = MessageTypeRobotStatus;
            advertise(n);
            RobotStatus status = content.value<RobotStatus>();
            robot_information_msgs::msg::RobotStatus s;

            s.battery = status.field1;
            s.battery_charging = status.field2;
            s.brake_active = status.field3;
            s.emergency_active = status.field5;
            s.drive_reversed = status.field4;
            s.turtle_factor = status.id; //TODO

            auto pub = std::static_pointer_cast<rclcpp::publisher::Publisher<robot_information_msgs::msg::RobotStatus> >(mPublishers.value(n));
            debug(LOG_BENCHMARK, "PUBLISHING robot_status", "id=%u", s.turtle_factor);
            pub->publish(s);
        }

    private:
        template<typename T>
        void advertise(communication::MessageType n)
        {
            if (!mNode)
                return;

            if (!mPublishers.contains(n))
            {
                debug(LOG_WARNING, "Ros2Publisher", "advertising %d on %s", n, qPrintable(RosTopics::fullTopic(n, "_")));

                auto pub = mNode->create_publisher<T>(RosTopics::fullTopic(n, "_").toStdString(),
                                                      Ros2QoSProfile::getProfile(n));
                mPublishers.insert(n, pub);
                debug(LOG_WARNING, "Ros2Publisher", "--advertised--, %u", (uint)(mPublishers.value(n)->get_queue_size()));
            }
        }

        rclcpp::Node::SharedPtr mNode;
        bool mStarted;
        QMap<MessageType, rclcpp::publisher::PublisherBase::SharedPtr> mPublishers;
    };

/*--------------------ROS2NodeImpl----------------------*/

    rclcpp::Node::SharedPtr getNode(QString name)
    {
        return rclcpp::node::Node::make_shared(qPrintable(name));
    }

    class Ros2NodeImpl : public QObject
    {
        Q_OBJECT
    private:
        RosInitializer mInitializer;

        rclcpp::Node::SharedPtr mNode;
        QString mName;
        Ros2Subscriber mSubscriber;
        Ros2Publisher mPublisher;

    signals:
        void messageReceived(communication::MessageType n, QVariant content);

    public:
        Ros2NodeImpl(QString name) :
            mInitializer(name),
            mName(name),
            mNode(getNode(name)),
            mSubscriber(mNode),
            mPublisher(mNode)
        {
            if (!mInitializer.initialized())
                return; //TODO - nothing to do

            connect(&mSubscriber, SIGNAL(messageReceived(communication::MessageType,QVariant)),
                    this, SIGNAL(messageReceived(communication::MessageType,QVariant)));
        }

        void start()
        {
            mSubscriber.start();
            mPublisher.start();
        }

        void advertise(MessageType n)
        {
            mPublisher.advertise(n);
        }

        void subscribe(MessageType n, bool subscribe)
        {
            mSubscriber.subscribe(n, subscribe);
        }

        void publishCmdVel(MoveBase mobileBase)
        {
            mPublisher.publishCmdVel(QVariant::fromValue(mobileBase));
        }

        void publishRobotStatus(RobotStatus status)
        {
            mPublisher.publishRobotStatus(QVariant::fromValue(status));
        }

        void publishRobotControl(RobotControl control)
        {
            mPublisher.publishRobotControl(QVariant::fromValue(control));
        }

        void publishByteMessage(int size)
        {
            mPublisher.publishByteMessage(QVariant::fromValue(size));
        }
    };
}

using namespace roscommunication;


/*--------------------ROS2Node----------------------*/


Ros2Node::Ros2Node(QString name) : d(new Ros2NodeImpl(name))
{
}

Ros2Node::~Ros2Node()
{
    //debug(LOG_WARNING, "RosNode2", "destruction");
    delete d;
}

void Ros2Node::start() { d->start(); }

void Ros2Node::publishCmdVel(MoveBase cmdVel) { return d->publishCmdVel(cmdVel); }
void Ros2Node::publishRobotStatus(RobotStatus status) { return d->publishRobotStatus(status); }
void Ros2Node::publishRobotControl(RobotControl control) { return d->publishRobotControl(control); }
void Ros2Node::publishByteMessage(int size) { return d->publishByteMessage(size); }

void Ros2Node::subscribe(MessageType n, bool subscribe) { return d->subscribe(n, subscribe); }
void Ros2Node::advertise(MessageType n) { d->advertise(n); }

#include "ros2node.moc"
