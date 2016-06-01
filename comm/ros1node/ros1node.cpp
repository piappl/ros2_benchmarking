#include <common/logging.h>

#include "ros1initializer.h"
#include "ros1masterchecker.h"
#include "ros1publisher.h"
#include "ros1subscriber.h"
#include "ros1node.h"

using namespace communication;

namespace roscommunication
{
    class Ros1NodeImpl : public QObject
    {
        Q_OBJECT
    private:
        Ros1Initializer mInitializer;
        Ros1MasterChecker mMasterChecker;
        NodeHandlePtr mNodeHandle;
        Ros1Publisher mPublisher;
        Ros1Subscriber mSubscriber;

    private slots:
        void masterStatusUpdate(bool isMasterUp)
        {
            //qDebug() << QThread::currentThreadId() << "masterStatusUpdate";

            if (isMasterUp)
            {
                if (mNodeHandle.isNull())
                {
                    debug(LOG_WARNING, "Ros1NodeImpl", "Creating node handle");
                    mNodeHandle = mMasterChecker.nodeHandle();
                    mPublisher.setNodeHandle(mNodeHandle);
                    mSubscriber.setNodeHandle(mNodeHandle);
                }
            }
            else
            {
                debug(LOG_WARNING, "Ros1NodeImpl", "ROS Master not up");
            }
            mPublisher.setMasterUp(isMasterUp);
            mSubscriber.setMasterUp(isMasterUp);
        }

        void onRosNotOk()
        {
            exit(1); //TODO - throw exception, catch in main, return
        }

    public:
        Ros1NodeImpl(Settings settings) :
            mInitializer(settings.nodeName),
            mPublisher(mNodeHandle),
            mSubscriber(mNodeHandle)
        {
            if (!mInitializer.initialized())
            {
                return; //TODO - nothing to do
            }
            connect(&mMasterChecker, SIGNAL(masterStatusUpdate(bool)), this,
                    SLOT(masterStatusUpdate(bool)), Qt::QueuedConnection);
            connect(&mMasterChecker, SIGNAL(rosNotOk()), this,
                    SLOT(onRosNotOk()), Qt::QueuedConnection);
            mMasterChecker.start();
        }

        ~Ros1NodeImpl()
        {
            //debug(LOG_WARNING, "Destruction", "of Ros1Impl");
            mMasterChecker.quit();
            mMasterChecker.wait();
        }

        void publishCmdVel(MoveBase mobileBase)
        {
            mPublisher.publishCmdVel(mobileBase);
        }

        void publishRobotStatus(RobotStatus status)
        {
            mPublisher.publishRobotStatus(status);
        }

        void publishRobotControl(RobotControl control)
        {
            mPublisher.publishRobotControl(control);
        }

        void publishByteMessage(int size)
        {
            mPublisher.publishByteMessage(size);
        }

        void subscribe(MessageType n)
        {
            mSubscriber.subscribe(n, true);
        }

        void unsubscribe(MessageType n)
        {
            mSubscriber.subscribe(n, false);
        }

        void advertise(MessageType n)
        {
            mPublisher.advertise(n);
        }

        const Ros1Publisher &publisher() const
        {
            return mPublisher;
        }

        const Ros1Subscriber &subscriber() const
        {
            return mSubscriber;
        }
    };
}

using namespace roscommunication;

Ros1Node::Ros1Node(Settings settings) : d(new Ros1NodeImpl(settings))
{
}

Ros1Node::~Ros1Node()
{
    //debug(LOG_WARNING, "RosNode", "destruction");
    delete d;
}

void Ros1Node::publishCmdVel(MoveBase cmdVel) { return d->publishCmdVel(cmdVel); }
void Ros1Node::publishRobotStatus(RobotStatus status) { return d->publishRobotStatus(status); }
void Ros1Node::publishRobotControl(RobotControl control) { return d->publishRobotControl(control); }
void Ros1Node::publishByteMessage(int size) { return d->publishByteMessage(size); }

void Ros1Node::advertise(MessageType n) { d->advertise(n); }

void Ros1Node::subscribe(MessageType n) { return d->subscribe(n); }
void Ros1Node::unsubscribe(MessageType n) { return d->unsubscribe(n); }

#include "ros1node.moc"
