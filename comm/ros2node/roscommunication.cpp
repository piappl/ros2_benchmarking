#include <roscommunication/roscommunication.h>
#include <roscommunication/rosnode.h>
#include <roscommunication/rosnodefactory.h>
#include <communication/connectioninformation.h>
#include <common/logging.h>

using namespace roscommunication;
using namespace psor;
using namespace communication;

RosCommunication::RosCommunication(CommunicationHost host, RosVersion version)
    : mVersion(version), mHost(host)
{
    RosNodeFactory factory;
    mRosNode = factory.makeNode(host, version);
    qRegisterMetaType<communication::CommunicationNotification>("communication::CommunicationNotification");
    connect(mRosNode.data(), SIGNAL(messageReceived(communication::CommunicationNotification,QVariant)),
            this, SLOT(incomingMessage(communication::CommunicationNotification,QVariant)));
}

CommunicationType RosCommunication::type() const
{
    return mVersion == ROS2 ? CommROS2 : CommROS;
}

void RosCommunication::incomingMessage(CommunicationNotification type, QVariant content)
{
    debug(LOG_DEBUG, "RosCommunication", "incoming Message %d", type);
    incomingNotification(type, content);
}

void RosCommunication::registerForNotification(CommunicationNotification notification)
{
    node()->subscribe(notification);
}

void RosCommunication::unregisterForNotification(CommunicationNotification notification)
{
    node()->unsubscribe(notification);
}

void RosCommunication::advertise(CommunicationNotification notification)
{
    node()->advertise(notification);
}

void RosCommunication::publish(CommunicationNotification notification, QVariant content)
{
    node()->publish(notification, content);
}

RosNodePtr RosCommunication::node() const
{
    return mRosNode;
}

bool RosCommunication::isActive() const
{
    return node()->isEnabled();
}

void RosCommunication::outgoingEvent(CommunicationEvent type, QVariant event)
{
    switch (type)
    {
        case DisconnectSenderEvent:
            node()->disableNamespace();
            return;
        case ConnectToPeerEvent:
            debug(LOG_WARNING, "RosCommunication", "ConnectToPeerEvent");
            node()->enableNamespace(event.value<ConnectionInformation>());
            incomingEvent(ConnectionSuccessEvent);
            return;
        case AutonomyStartEvent:
        case AutonomyStopEvent:
        case AutonomyPauseEvent:
            return; //TODO
            //return node()->autonomyEvent(type, event);
        default:
            return;
    }
}

bool RosCommunication::isReadyToSend() const
{
    return isActive();
}

QHostAddress RosCommunication::remoteAddress() const
{
    return QHostAddress(); //TODO
}
