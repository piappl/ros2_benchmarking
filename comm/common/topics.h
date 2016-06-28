#ifndef ROSTOPICS_H
#define ROSTOPICS_H

#include <QString>
#include <string>

#include "messagetypes.h"
#include "logging.h"

namespace communication
{
    class Topics
    {
        private:
            static QString join() { return "/"; }

        public:
            static QString topic(communication::MessageType n)
            {
                switch (n)
                {
                    case communication::MessageTypeRobotAlarm:
                        return "robot_alarm";
                    case communication::MessageTypeRobotSensor:
                        return "robot_sensor";
                    case communication::MessageTypeRobotControl:
                    default:
                        return "robot_control";
                }
            }

            static QString namespaceString(QString glue = join())
            {
                return QString("robot") + glue;
            }

            static QString fullTopic(communication::MessageType n, QString glue = join())
            {
                return namespaceString(glue) + topic(n);
            }
    };
}

#endif //ROSTOPICS_H
