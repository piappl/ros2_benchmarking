#include <common/logging.h>
#include <common/communicationutils.h>
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

    mRobotControlTimer.setInterval(mConfig->timerInterval(TimerRobotControl));
    mRobotAlarmTimer.setInterval(mConfig->timerInterval(TimerRobotAlarm));
    mRobotSensorTimer.setInterval(mConfig->timerInterval(TimerRobotSensor));

    mStartDelayTimer.setSingleShot(true);
    mQuitDelayTimer.setSingleShot(true);
    mTestTimer.setSingleShot(true);

    mRobotControlTimer.setSingleShot(false);
    mRobotAlarmTimer.setSingleShot(false);
    mRobotSensorTimer.setSingleShot(false);

    connect(&mStartDelayTimer, SIGNAL(timeout()), this, SLOT(startTest()));
    connect(&mQuitDelayTimer, SIGNAL(timeout()), this, SIGNAL(quit()));
    connect(&mTestTimer, SIGNAL(timeout()), this, SLOT(finishTest()));

    connect(&mRobotControlTimer, SIGNAL(timeout()), this, SLOT(publishRobotControl()));
    connect(&mRobotAlarmTimer, SIGNAL(timeout()), this, SLOT(publishRobotAlarm()));
    connect(&mRobotSensorTimer, SIGNAL(timeout()), this, SLOT(publishRobotSensor()));

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
    if (mConfig->publishes().contains(MessageTypeRobotControl))
    {
        mRobotControlTimer.start();
    }
    if (mConfig->publishes().contains(MessageTypeRobotAlarm))
    {
        mRobotAlarmTimer.start();
    }
    if (mConfig->publishes().contains(MessageTypeRobotSensor))
    {
        mRobotSensorTimer.start();
    }
    mTestTimer.start();
}

void TestRunner::finishTest()
{
    debug(LOG_BENCHMARK, "TestRunner", "Test finished, will close after delay");
    mRobotControlTimer.stop();
    mRobotAlarmTimer.stop();
    mRobotSensorTimer.stop();
    mQuitDelayTimer.start();
}

void TestRunner::publishRobotControl()
{
    static int i = 0;
    RobotControl msg;
    msg.id = i++;
    msg.x = 1;
    msg.y = 2;
    msg.z = 3;
    mNode->publishRobotControl(msg);
}

void TestRunner::publishRobotAlarm()
{
    static int i = 0;
    RobotAlarm msg;
    msg.id = i++;
    msg.alarm1 = 1;
    msg.alarm2 = 2;
    mNode->publishRobotAlarm(msg);
}

void TestRunner::publishRobotSensor()
{
    static int i = 0;
    RobotSensor msg;
    msg.id = i++;
    CommunicationUtils::fillRandomVector(mConfig->byteMessageSize(), msg.data);
    mNode->publishRobotSensor(msg);
}
