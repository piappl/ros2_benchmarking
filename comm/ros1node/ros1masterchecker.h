#ifndef ROS1MASTERCHECKER_H
#define ROS1MASTERCHECKER_H

#include <QThread>
#include "ros1fwd.h"

namespace roscommunication
{
    class Ros1MasterChecker : public QThread
    {
        Q_OBJECT
    signals:
        void masterStatusUpdate(bool up);
        void rosNotOk();

    public:
        void stop();
        void run();
        NodeHandlePtr nodeHandle() const;

    private slots:
        void checkMasterUp();

    private:
        NodeHandlePtr mNodeHandle;      
        AsyncSpinnerPtr mSpinner;
    };
}

#endif // ROS1MASTERCHECKER_H
