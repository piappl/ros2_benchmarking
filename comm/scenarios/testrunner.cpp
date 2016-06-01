#include <common/logging.h>
#include "testrunner.h"

using namespace communication;

//TODO - refactor this

TestRunner::TestRunner(ConfigParser *p, NodeInterfacePtr node)
    : mConfig(p), mNode(node)
{
    debug(LOG_BENCHMARK, "TestRunner", "Initializing, test will start after delay");

    initTimers();
    advertise();
    subscribe();
}

void TestRunner::initTimers()
{
    mStartDelayTimer.setInterval(mConfig->timerInterval(TimerStartDelay));
    mQuitDelayTimer.setInterval(mConfig->timerInterval(TimerQuitDelay));
    mTestTimer.setInterval(mConfig->timerInterval(TimerTestingTime));

    mCmdVelTimer.setInterval(mConfig->timerInterval(TimerCmdVel));
    mRobotControlTimer.setInterval(mConfig->timerInterval(TimerRobotControl));
    mRobotStatusTimer.setInterval(mConfig->timerInterval(TimerRobotStatus));
    mBytesTimer.setInterval(mConfig->timerInterval(TimerBytes));

    mStartDelayTimer.setSingleShot(true);
    mQuitDelayTimer.setSingleShot(true);
    mTestTimer.setSingleShot(true);

    mCmdVelTimer.setSingleShot(false);
    mRobotControlTimer.setSingleShot(false);
    mRobotStatusTimer.setSingleShot(false);
    mBytesTimer.setSingleShot(false);

    connect(&mStartDelayTimer, SIGNAL(timeout()), this, SLOT(startTest()));
    connect(&mQuitDelayTimer, SIGNAL(timeout()), this, SIGNAL(quit()));
    connect(&mTestTimer, SIGNAL(timeout()), this, SLOT(finishTest()));

    connect(&mCmdVelTimer, SIGNAL(timeout()), this, SLOT(publishCmdVel()));
    connect(&mRobotControlTimer, SIGNAL(timeout()), this, SLOT(publishRobotControl()));
    connect(&mRobotStatusTimer, SIGNAL(timeout()), this, SLOT(publishRobotStatus()));
    connect(&mBytesTimer, SIGNAL(timeout()), this, SLOT(publishBytes()));

    mStartDelayTimer.start(); //We are starting the test after the delay!
}

void TestRunner::advertise()
{
    foreach (MessageType t, mConfig->publishes())
    {
        mNode->advertise(t);
    }
}

void TestRunner::subscribe()
{
    foreach (MessageType t, mConfig->subscribes())
    {
        mNode->subscribe(t);
    }
    mNode->start();
}

void TestRunner::unsubscribe()
{
    foreach (MessageType t, mConfig->subscribes())
    {
        mNode->unsubscribe(t);
    }
}

void TestRunner::startTest()
{
    debug(LOG_BENCHMARK, "TestRunner", "Starting test");
    if (mConfig->publishes().contains(MessageTypeCmdVel))
    {
        mCmdVelTimer.start();
    }
    if (mConfig->publishes().contains(MessageTypeRobotControl))
    {
        mRobotControlTimer.start();
    }
    if (mConfig->publishes().contains(MessageTypeRobotStatus))
    {
        mRobotStatusTimer.start();
    }
    if (mConfig->publishes().contains(MessageTypeBytes))
    {
        mBytesTimer.start();
    }
    mTestTimer.start();
}

void TestRunner::finishTest()
{
    debug(LOG_BENCHMARK, "TestRunner", "Test finished, will close after delay");
    unsubscribe();
    mCmdVelTimer.stop();
    mRobotControlTimer.stop();
    mRobotStatusTimer.stop();
    mBytesTimer.stop();
    mQuitDelayTimer.start();
}

void TestRunner::publishCmdVel()
{
    static int i = 0;
    MoveBase base;
    base.id = i;
    base.x = 1;
    base.y = 2;
    base.z = 3;
    i++;
    mNode->publishCmdVel(base);
}

void TestRunner::publishRobotControl()
{
    static int i = 0;
    RobotControl rc;
    rc.id = i;
    rc.field1 = 1;
    rc.field2 = 2;
    i++;
    mNode->publishRobotControl(rc);
}

void TestRunner::publishRobotStatus()
{
    static int i = 0;
    RobotStatus rs;
    rs.id = i;
    rs.field1 = 1;
    rs.field2 = 2;
    i++;
    mNode->publishRobotStatus(rs);
}

void TestRunner::publishBytes()
{
    mNode->publishByteMessage(mConfig->byteMessageSize());
}
