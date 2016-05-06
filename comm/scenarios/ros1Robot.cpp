#include <QtCore/QCoreApplication>
#include "ros1Robot.h"

using namespace roscommunication;
using namespace communication;

int main(int argc, char *argv[])
{
    //TODO - parse params, apply QoS and stuff
    QCoreApplication a(argc, argv);

    Ros1Node node("Ros1Robot");
    node.advertise(MessageTypeBytes);
    node.advertise(MessageTypeRobotStatus);
    node.subscribe(MessageTypeCmdVel);
    node.subscribe(MessageTypeRobotControl);

    a.exec();
}
