#ifndef ROSTOPICS_H
#define ROSTOPICS_H

#include <QString>
#include <string>

#include "messagetypes.h"
#include "logging.h"

namespace roscommunication
{
    class RosTopics
    {
        private:
            static QString join() { return "/"; }

        public:
            static QString topic(communication::MessageType n)
            {
                switch (n)
                {
                case communication::MessageTypeRobotControl:
                    return "robot_control";
                case communication::MessageTypeRobotStatus:
                    return "robot_status";
                case communication::MessageTypeBytes:
                    return "robot_bytes";
                case communication::MessageTypeCmdVel:
                default:
                    return "cmd_vel";
                }
            }

            static QString namespaceString(communication::MessageType n, QString glue = join())
            {
                return QString("robot") + glue;
            }

            static QString fullTopic(communication::MessageType n,
                                     QString glue = join())
            {
                return namespaceString(n, glue) + topic(n);
            }
    };
}

#endif //ROSTOPICS_H
