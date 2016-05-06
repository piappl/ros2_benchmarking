#include <QtCore/QCoreApplication>
#include "ros2Robot.h"

using namespace roscommunication;
using namespace communication;

int main(int argc, char *argv[])
{
    //TODO - parse params, apply QoS and stuff
    QCoreApplication a(argc, argv);

    Ros2Node node("Ros2Robot");
    node.advertise(MessageTypeCmdVel);
    node.advertise(MessageTypeRobotControl);
    node.subscribe(MessageTypeBytes);
    node.subscribe(MessageTypeRobotStatus);

    a.exec();
}
