#include <QtCore/QCoreApplication>
#include <QTimer>
#include "ros1Console.h"

using namespace roscommunication;
using namespace communication;

//TODO - refactor this, much common for other cases

namespace
{   //TODO - cmd line args with defaults
    const int kCmdVelInterval = 250;
    const int kRobotControlInterval = 500;
    const int kStartAfter = 2000; //ms
    const int kFinishAfter = 20000; //ms
    const int kQuitDelay = 5000;
}

Ros1Console::Ros1Console() : mNode("Ros1Console")
{
    mNode.advertise(MessageTypeCmdVel);
    mNode.advertise(MessageTypeRobotControl);
    mNode.subscribe(MessageTypeBytes);
    mNode.subscribe(MessageTypeRobotStatus);
    mCVTimer.setInterval(kStartAfter); //Initial
    mRCTimer.setInterval(kStartAfter); //Initial
    mWholeTestTimer.setInterval(kFinishAfter);
    mQuitDelayTimer.setInterval(kQuitDelay);
    mCVTimer.setSingleShot(false);
    mRCTimer.setSingleShot(false);
    connect(&mCVTimer, SIGNAL(timeout()), this, SLOT(publishCV()));
    connect(&mRCTimer, SIGNAL(timeout()), this, SLOT(publishRC()));
    connect(&mWholeTestTimer, SIGNAL(timeout()), this, SLOT(finishTest()));
    connect(&mQuitDelayTimer, SIGNAL(timeout()), this, SIGNAL(quit()));
    mCVTimer.start();
    mRCTimer.start();
    mWholeTestTimer.start();
}

void Ros1Console::publishCV()
{
    mCVTimer.setInterval(kCmdVelInterval);
    printf("Publishing CV\n");
    static int i = 0;
    MoveBase base;
    base.id = i;
    base.x = 1;
    base.y = 2;
    base.z = 3;
    i++;
    mNode.publishCmdVel(base);
}

void Ros1Console::publishRC()
{
    mRCTimer.setInterval(kRobotControlInterval);
    printf("Publishing RC\n");
    static int i = 0;
    RobotControl rc;
    rc.id = i;
    rc.field1 = 1;
    rc.field2 = 2;
    i++;
    mNode.publishRobotControl(rc);
}

void Ros1Console::finishTest()
{
    printf("Finalising test\n");
    mRCTimer.stop();
    mCVTimer.stop();
    mNode.subscribe(MessageTypeBytes, false);
    mNode.subscribe(MessageTypeRobotStatus, false);
    mQuitDelayTimer.start();
}

int main(int argc, char *argv[])
{
    //TODO - parse params, apply QoS and stuff
    printf("Test starting!\n");
    QCoreApplication a(argc, argv);
    Ros1Console console;
    QObject::connect(&console, SIGNAL(quit()), &a, SLOT(quit()));

    a.exec();
    printf("Test finished!\n");
}
