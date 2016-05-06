#ifndef ROS1INITIALIZER_H
#define ROS1INITIALIZER_H

#include <QString>

namespace roscommunication
{
    class Ros1Initializer
    {   //Responsible for ROS init, construct before using ROS
        public:
            bool initialized() const;
            Ros1Initializer(QString name);
        private:
            bool mInitialized;
    };
}

#endif // ROS1INITIALIZER_H
