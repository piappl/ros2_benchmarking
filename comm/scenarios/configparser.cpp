#include <QFile>
#include <common/logging.h>
#include "configparser.h"

using namespace communication;

//TODO - refactor this (enum -> string dict etc)

namespace
{   //defaults in ms
    const int kCmdVelInterval = 250;
    const int kRobotControlInterval = 500;
    const int kBytesInterval = 100;
    const int kRobotStatusInterval = 300;
    const int kStartDelay = 2000;
    const int kTestingTime = 20000;
    const int kQuitDelay = 5000;

    const QString kDefaultQoS = "default";
}

ConfigParser::ConfigParser(QString filename)
    : mSettings(filename, QSettings::IniFormat)
{   //TODO - does not check for argument sanity
    parseTimersSection();
    parsePublish();
    parseSubscribe();
    parseQoS();
}

void ConfigParser::parseTimersSection()
{
    bool ok;    //add type checking!
    mTimersIntervals[TimerStartDelay] = mSettings.value("timers/startdelay", kStartDelay).toInt(&ok);
    mTimersIntervals[TimerQuitDelay] = mSettings.value("timers/quitdelay", kQuitDelay).toInt(&ok);
    mTimersIntervals[TimerTestingTime] = mSettings.value("timers/testingtime", kTestingTime).toInt(&ok);

    mTimersIntervals[TimerCmdVel] = mSettings.value("timers/cmdvel", kCmdVelInterval).toInt(&ok);
    mTimersIntervals[TimerRobotControl] = mSettings.value("timers/robotcontrol", kRobotControlInterval).toInt(&ok);
    mTimersIntervals[TimerRobotStatus] = mSettings.value("timers/robotstatus", kRobotStatusInterval).toInt(&ok);
    mTimersIntervals[TimerBytes] = mSettings.value("timers/bytes", kBytesInterval).toInt(&ok);
}

void ConfigParser::parsePublish()
{   //TODO - refactor, smart
    if (1 == mSettings.value("publish/cmdvel", 0))
    {
        mPublish.append(MessageTypeCmdVel);
    }
    if (1 == mSettings.value("publish/robotcontrol", 0))
    {
        mPublish.append(MessageTypeRobotControl);
    }
    if (1 == mSettings.value("publish/robotstatus", 0))
    {
        mPublish.append(MessageTypeRobotStatus);
    }
    if (1 == mSettings.value("publish/bytes", 0))
    {
        mPublish.append(MessageTypeBytes);
    }
}

void ConfigParser::parseSubscribe()
{   //TODO - common with above
    if (1 == mSettings.value("subscribe/cmdvel", 0))
    {
        mSubscribe.append(MessageTypeCmdVel);
    }
    if (1 == mSettings.value("subscribe/robotcontrol", 0))
    {
        mSubscribe.append(MessageTypeRobotControl);
    }
    if (1 == mSettings.value("subscribe/robotstatus", 0))
    {
        mSubscribe.append(MessageTypeRobotStatus);
    }
    if (1 == mSettings.value("subscribe/bytes", 0))
    {
        mSubscribe.append(MessageTypeBytes);
    }
}

void ConfigParser::parseQoS()
{
    mQoS[MessageTypeCmdVel] = mSettings.value("QoS/cmdVel", kDefaultQoS).toString();
    mQoS[MessageTypeRobotControl] = mSettings.value("QoS/robotcontrol", kDefaultQoS).toString();
    mQoS[MessageTypeRobotStatus] = mSettings.value("QoS/robotstatus", kDefaultQoS).toString();
    mQoS[MessageTypeBytes] = mSettings.value("QoS/bytes", kDefaultQoS).toString();
}

QString ConfigParser::getQoS(communication::MessageType t) const
{
    return mQoS.value(t, kDefaultQoS);
}

QList<communication::MessageType> ConfigParser::subscribes() const
{
    return mSubscribe;
}

QList<communication::MessageType> ConfigParser::publishes() const
{
    return mPublish;
}

int ConfigParser::timerInterval(TimerType t) const
{
    return mTimersIntervals.value(t);
}
