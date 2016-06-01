#ifndef CONFIGPARSER_H
#define CONFIGPARSER_H

#include <QString>
#include <QMap>
#include <QSettings>
#include <common/messagetypes.h>
#include <common/settings.h>
#include "timersenum.h"

class ConfigParser
{
public:
    ConfigParser(QString filepath);

    QList<communication::MessageType> subscribes() const;
    QList<communication::MessageType> publishes() const;
    communication::Settings nodeConfig() const;
    int timerInterval(TimerType t) const;
    int byteMessageSize() const;

private:
    void parseIDsSection();
    void parseTimersSection();
    void parsePublish();
    void parseSubscribe();
    void parseQoS();
    void parseOther();

    QSettings mSettings;
    communication::Settings mNodeConfig;
    QMap<TimerType, int> mTimersIntervals;
    QList<communication::MessageType> mSubscribe;
    QList<communication::MessageType> mPublish;
    int mByteMessageSize;
};

#endif //CONFIGPARSER_H
