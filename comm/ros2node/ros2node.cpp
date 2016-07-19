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
#include <common/topics.h>
#include "ros2node.h"
#include "ros2messages.h"
#include "ros2messagetovariant.h"
#include "ros2subscriptionlistenerinterface.h"
#include "ros2qosprofile.h"
#ifdef FASTRTPS
#include "fastrtps/utils/RTPSLog.h"
#endif

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
                qRegisterMetaType<communication::MessageType>("communication::MessageType");
                debug(LOG_WARNING, "RosInitializer", "DDS: %s", rmw_get_implementation_identifier());
#ifdef FASTRTPS
                eprosima::Log::setVerbosity(eprosima::VERB_INFO);
#endif
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
        Ros2SubscriptionListener(communication::MessageType type,
                                 rclcpp::Node::SharedPtr node,
                                 QoSSetting qos)
            : Ros2SubscriptionListenerInterface(type), mNode(node)
        {
            subscribe(qos);
        }

        void processMessage(const typename T::SharedPtr msg)
        {
           //debug(LOG_WARNING, "Ros2node", "process message %d starts", messageType());
           if (isStopped())
           {   //TODO - upgrade. For now, just don't process
               debug(LOG_WARNING, "Ros2SubscriptionListener", "Listener is stopped");
               return;
           }

           QVariant content = ros2MessageToVariant(msg);
           emit messageReceived(messageType(), content);
        }

    private:
        rclcpp::Node::SharedPtr mNode;
        typename rclcpp::subscription::Subscription<T>::SharedPtr mSubscription;

        void subscribe(QoSSetting qos)
        {
            auto callback =
              [this](const typename T::SharedPtr msg) -> void
              {
                this->processMessage(msg);
              };

            QString fullTopicName = Topics::fullTopic(messageType(), "_");

            debug(LOG_WARNING, "Ros2SubscriptionListener", "Subscribe |%s| mNode %lld QoS[%s]",
                  qPrintable(fullTopicName), (qint64)mNode.get(), Ros2QoSProfile::profileDescription(qos.profile).data());

            mSubscription = mNode->create_subscription<T>
                    (fullTopicName.toStdString(), callback, Ros2QoSProfile::getProfile(qos.profile));
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

        void subscribe(communication::MessageType n, QoSSetting s)
        {
            //debug(LOG_WARNING, "Ros2ListenerThread", "subscribe attempt to %d", n);
            if (!mListeners.contains(n))
            {
                Ros2SubscriptionListenerInterfacePtr construct = constructListener(n, s);
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

        Ros2SubscriptionListenerInterfacePtr constructListener(communication::MessageType n, QoSSetting qos)
        {
            //debug(LOG_WARNING, "Ros2Subscriber", "constructing listener %d", n);

            Ros2SubscriptionListenerInterfacePtr listenerPtr;
            switch (n)
            {
            case MessageTypeRobotControl:
                listenerPtr.reset(new Ros2SubscriptionListener<ros2eval_msgs::msg::RobotControl>(n, mNode, qos));
                break;
            case MessageTypeRobotAlarm:
                listenerPtr.reset(new Ros2SubscriptionListener<ros2eval_msgs::msg::RobotAlarm>(n, mNode, qos));
                break;
            case MessageTypeRobotSensor:
                listenerPtr.reset(new Ros2SubscriptionListener<ros2eval_msgs::msg::RobotSensor>(n, mNode, qos));
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
        Ros2Subscriber(rclcpp::Node::SharedPtr node, QoSSettings s)
            : mNode(node), mListener(node), mQoS(s)
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

        void subscribe(communication::MessageType n)
        {
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
            mListener.subscribe(n, mQoS.value(n));
        }

        void unsubscribe(communication::MessageType /*n*/)
        {
            //TODO - implement
            debug(LOG_ERROR, "Ros2Subscriber", "Unsubscribe: not implemented (yet)!");
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
                mListener.subscribe(n, mQoS.value(n));
            }
            startListener();
        }

        QSet<communication::MessageType> mPlannedSubscriptions;
        rclcpp::Node::SharedPtr mNode;
        Ros2ListenerThread mListener;
        QoSSettings mQoS;
    };
    typedef QSharedPointer<Ros2Subscriber> Ros2SubscriberPtr;


/*--------------------ROS2Publisher---------------------*/


    class Ros2Publisher
    {
    public:
        Ros2Publisher(rclcpp::Node::SharedPtr node, QoSSettings settings)
            : mQoS(settings), mNode(node), mStarted(false) {}

        void advertise(MessageType n)
        {
            switch (n)
            {
                case MessageTypeRobotControl:
                    return advertise<ros2eval_msgs::msg::RobotControl>(n);
                case MessageTypeRobotAlarm:
                    return advertise<ros2eval_msgs::msg::RobotAlarm>(n);
                case MessageTypeRobotSensor:
                    return advertise<ros2eval_msgs::msg::RobotSensor>(n);
                default:
                    debug(LOG_ERROR, "Ros2Publisher", "advertise: not implemented topic for %d", n);
                    break;
            }
        }

        void start()
        {
            mStarted = true;
        }

        void publishRobotSensor(QVariant content)
        {
            if (!mStarted)
            {
                return;
            }
            if (!mNode)
            {
                debug(LOG_ERROR, "Ros2Publisher", "ERROR! No node - cannot publish");
                return;
            }

            MessageType n = MessageTypeRobotSensor;
            advertise(n);
            communication::RobotSensor msg = content.value<communication::RobotSensor>();
            ros2eval_msgs::msg::RobotSensor sensor;
            sensor.id = msg.id;
            sensor.data = msg.data;
            auto pub = std::static_pointer_cast<rclcpp::publisher::Publisher<ros2eval_msgs::msg::RobotSensor>>(mPublishers.value(n));
            debug(LOG_BENCHMARK, "PUBLISHING RobotSensor", "id=%d, size=%lu", msg.id, msg.data.size());
            try
            {
                pub->publish(sensor);
            }
            catch (std::exception& e)
            {
                debug(LOG_ERROR, "Ros2Publisher", "%s", e.what());
            }
        }

        void publishRobotControl(QVariant content)
        {
            if (!mStarted)
            {
                return;
            }
            if (!mNode)
            {
                debug(LOG_ERROR, "Ros2Publisher", "ERROR! No node - cannot publish");
                return;
            }

            MessageType n = MessageTypeRobotControl;
            advertise(n);
            communication::RobotControl msg = content.value<communication::RobotControl>();
            ros2eval_msgs::msg::RobotControl control;
            control.id = msg.id;
            control.x = msg.x;
            control.z = msg.z;
            control.z = msg.id;
            debug(LOG_BENCHMARK, "PUBLISHING RobotControl", "id=%d, size=%lu", msg.id, sizeof(ros2eval_msgs::msg::RobotControl));
            auto pub = std::static_pointer_cast<rclcpp::publisher::Publisher<ros2eval_msgs::msg::RobotControl> >(mPublishers.value(n));
            try
            {
                pub->publish(control);
            }
            catch (std::exception& e)
            {
                debug(LOG_ERROR, "Ros2Publisher", "%s", e.what());
            }
        }

        void publishRobotAlarm(QVariant content)
        {
            if (!mStarted)
            {
                return;
            }
            if (!mNode)
            {
                debug(LOG_ERROR, "Ros2Publisher", "ERROR! No node - cannot publish");
                return;
            }

            MessageType n = MessageTypeRobotAlarm;
            advertise(n);
            communication::RobotAlarm msg = content.value<communication::RobotAlarm>();
            ros2eval_msgs::msg::RobotAlarm alarm;
            alarm.id = msg.id;
            alarm.alarm1 = msg.alarm1;
            alarm.alarm2 = msg.alarm2;
            auto pub = std::static_pointer_cast<rclcpp::publisher::Publisher<ros2eval_msgs::msg::RobotAlarm> >(mPublishers.value(n));
            debug(LOG_BENCHMARK, "PUBLISHING RobotAlarm", "id=%u, size=%lu", alarm.id, sizeof(ros2eval_msgs::msg::RobotAlarm));
            try
            {
                pub->publish(alarm);
            }
            catch (std::exception& e)
            {
                debug(LOG_ERROR, "Ros2Publisher", "%s", e.what());
            }
        }

    private:
        template<typename T>
        void advertise(communication::MessageType n)
        {
            if (!mNode)
                return;

            if (!mPublishers.contains(n))
            {
                debug(LOG_WARNING, "Ros2Publisher", "advertising %d on %s QoS[%s]", n, qPrintable(Topics::fullTopic(n, "_")), Ros2QoSProfile::profileDescription(mQoS.value(n).profile).data());

                auto pub = mNode->create_publisher<T>(Topics::fullTopic(n, "_").toStdString(),
                                                      Ros2QoSProfile::getProfile(mQoS.value(n).profile));
                mPublishers.insert(n, pub);
                //debug(LOG_WARNING, "Ros2Publisher", "--advertised--, %u", (uint)(mPublishers.value(n)->get_queue_size()));
            }
        }

        QoSSettings mQoS;
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
        QString mName;
        rclcpp::Node::SharedPtr mNode;
        Ros2Subscriber mSubscriber;
        Ros2Publisher mPublisher;

    signals:
        void messageReceived(communication::MessageType n, QVariant content);

    public:
        Ros2NodeImpl(Settings s) :
            mInitializer(s.nodeName),
            mName(s.nodeName),
            mNode(getNode(s.nodeName)),
            mSubscriber(mNode, s.qos),
            mPublisher(mNode, s.qos)
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

        void subscribe(MessageType n)
        {
            mSubscriber.subscribe(n);
        }

        void unsubscribe(MessageType n)
        {
            mSubscriber.unsubscribe(n);
        }

        void publishRobotControl(communication::RobotControl control)
        {
            mPublisher.publishRobotControl(QVariant::fromValue(control));
        }

        void publishRobotAlarm(communication::RobotAlarm alarm)
        {
            mPublisher.publishRobotAlarm(QVariant::fromValue(alarm));
        }

        void publishRobotSensor(communication::RobotSensor sensor)
        {
            mPublisher.publishRobotSensor(QVariant::fromValue(sensor));
        }
    };
}

using namespace roscommunication;


/*--------------------ROS2Node----------------------*/


Ros2Node::Ros2Node(Settings s) : d(new Ros2NodeImpl(s))
{
}

Ros2Node::~Ros2Node()
{
    //debug(LOG_WARNING, "RosNode2", "destruction");
    delete d;
}

void Ros2Node::start() { d->start(); }

void Ros2Node::publishRobotControl(communication::RobotControl control) { return d->publishRobotControl(control); }
void Ros2Node::publishRobotAlarm(communication::RobotAlarm alarm) { return d->publishRobotAlarm(alarm); }
void Ros2Node::publishRobotSensor(communication::RobotSensor sensor) { return d->publishRobotSensor(sensor); }

void Ros2Node::subscribe(MessageType n) { return d->subscribe(n); }
void Ros2Node::unsubscribe(MessageType n) { return d->unsubscribe(n); }

void Ros2Node::advertise(MessageType n) { d->advertise(n); }

#include "ros2node.moc"
