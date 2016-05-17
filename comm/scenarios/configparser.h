#ifndef CONFIGPARSER_H
#define CONFIGPARSER_H

#include <QString>
#include <QMap>
#include <QSettings>
#include <common/messagetypes.h>
#include "timersenum.h"

class ConfigParser
{
public:
    ConfigParser(QString filepath);

    QString getQoS(communication::MessageType t) const;
    QList<communication::MessageType> subscribes() const;
    QList<communication::MessageType> publishes() const;
    int timerInterval(TimerType t) const;
    int byteMessageSize() const;

private:
    void parseTimersSection();
    void parsePublish();
    void parseSubscribe();
    void parseQoS();
    void parseOther();

    QSettings mSettings;
    QMap<TimerType, int> mTimersIntervals;
    QMap<communication::MessageType, QString> mQoS;
    QList<communication::MessageType> mSubscribe;
    QList<communication::MessageType> mPublish;
    int mByteMessageSize;
};

#endif //CONFIGPARSER_H
