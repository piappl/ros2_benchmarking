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
    const int kByteMessageSize = 1000;
    const int kDomainID = 0;

    const QString kDefaultQoS = "default";
    const QString kDefaultNodeName = "node";

    QoSSetting qosDict(QString qosstring)
    {
        QoSSetting setting;
        if (qosstring == "sensor")
        {
            setting.profile = QoSProfileSensor;
        }
        else if (qosstring == "alarm")
        {
            setting.profile = QoSProfileAlarm;
        }
        else if (qosstring == "control")
        {
            setting.profile = QoSProfileControl;
        }
        else if (qosstring == "status")
        {
            setting.profile = QoSProfileStatus;
        }
        else
        {
            setting.profile = QoSProfileDefault;
        }
        return setting;
    }
}

ConfigParser::ConfigParser(QString filename)
    : mSettings(filename, QSettings::IniFormat),
      mByteMessageSize(kByteMessageSize)
{   //TODO - does not check for argument sanity
    parseIDsSection();
    parseTimersSection();
    parsePublish();
    parseSubscribe();
    parseQoS();
    parseOther();
}

void ConfigParser::parseIDsSection()
{
    bool ok;    //add type checking!
    mNodeConfig.domainID = mSettings.value("ids/domainid", kDomainID).toInt(&ok);
    mNodeConfig.nodeName = mSettings.value("ids/nodename", kDefaultNodeName).toString();
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
    mNodeConfig.qos[MessageTypeCmdVel] = qosDict(mSettings.value("QoS/cmdvel", kDefaultQoS).toString());
    mNodeConfig.qos[MessageTypeRobotControl] = qosDict(mSettings.value("QoS/robotcontrol", kDefaultQoS).toString());
    mNodeConfig.qos[MessageTypeRobotStatus] = qosDict(mSettings.value("QoS/robotstatus", kDefaultQoS).toString());
    mNodeConfig.qos[MessageTypeBytes] = qosDict(mSettings.value("QoS/bytes", kDefaultQoS).toString());
}

void ConfigParser::parseOther()
{
    bool ok; //TODO add check
    mSettings.value("other/bytemessagesize", kByteMessageSize).toInt(&ok);
}

Settings ConfigParser::nodeConfig() const
{
    return mNodeConfig;
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

int ConfigParser::byteMessageSize() const
{
    return mByteMessageSize;
}
