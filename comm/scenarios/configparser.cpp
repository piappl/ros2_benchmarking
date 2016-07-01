#include <QFile>
#include <common/logging.h>
#include "configparser.h"

using namespace communication;

//TODO - refactor this (enum -> string dict etc)

namespace
{   //defaults in ms
    const int kRobotControlInterval = 250;
    const int kRobotAlarmInterval = 500;
    const int kRobotSensorInterval = 100;
    const int kLaunchDelay = 500;
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
    mTimersIntervals[TimerLaunchDelay] = mSettings.value("timers/launchdelay", kLaunchDelay).toInt(&ok);
    mTimersIntervals[TimerStartDelay] = mSettings.value("timers/startdelay", kStartDelay).toInt(&ok);
    mTimersIntervals[TimerQuitDelay] = mSettings.value("timers/quitdelay", kQuitDelay).toInt(&ok);
    mTimersIntervals[TimerTestingTime] = mSettings.value("timers/testingtime", kTestingTime).toInt(&ok);
    mTimersIntervals[TimerRobotControl] = mSettings.value("timers/control", kRobotControlInterval).toInt(&ok);
    mTimersIntervals[TimerRobotAlarm] = mSettings.value("timers/alarm", kRobotAlarmInterval).toInt(&ok);
    mTimersIntervals[TimerRobotSensor] = mSettings.value("timers/sensor", kRobotSensorInterval).toInt(&ok);
}

void ConfigParser::parsePublish()
{   //TODO - refactor, smart
    if (1 == mSettings.value("publish/control", 0))
    {
        mPublish.append(MessageTypeRobotControl);
    }
    if (1 == mSettings.value("publish/alarm", 0))
    {
        mPublish.append(MessageTypeRobotAlarm);
    }
    if (1 == mSettings.value("publish/sensor", 0))
    {
        mPublish.append(MessageTypeRobotSensor);
    }
}

void ConfigParser::parseSubscribe()
{   //TODO - common with above
    if (1 == mSettings.value("subscribe/control", 0))
    {
        mSubscribe.append(MessageTypeRobotControl);
    }
    if (1 == mSettings.value("subscribe/alarm", 0))
    {
        mSubscribe.append(MessageTypeRobotAlarm);
    }
    if (1 == mSettings.value("subscribe/sensor", 0))
    {
        mSubscribe.append(MessageTypeRobotSensor);
    }
}

void ConfigParser::parseQoS()
{
    mNodeConfig.qos[MessageTypeRobotControl] = qosDict(mSettings.value("QoS/control", kDefaultQoS).toString());
    mNodeConfig.qos[MessageTypeRobotAlarm] = qosDict(mSettings.value("QoS/alarm", kDefaultQoS).toString());
    mNodeConfig.qos[MessageTypeRobotSensor] = qosDict(mSettings.value("QoS/sensor", kDefaultQoS).toString());
}

void ConfigParser::parseOther()
{
    bool ok; //TODO add check
    mByteMessageSize = mSettings.value("other/bytemessagesize", kByteMessageSize).toInt(&ok);
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
