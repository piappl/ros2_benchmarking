#include <QTimer>
#include <QDebug>

#include <ros/master.h>
#include <ros/ros.h>

#include <common/logging.h>
#include "ros1masterchecker.h"

using namespace roscommunication;

void Ros1MasterChecker::run()
{
    //qDebug() << QThread::currentThreadId() << "run";
    QTimer checkerTimer;
    const int interval = 1000;
    connect(&checkerTimer, SIGNAL(timeout()), this, SLOT(checkMasterUp()), Qt::DirectConnection);
    checkerTimer.setInterval(interval);
    checkerTimer.setSingleShot(false);
    checkerTimer.start();
    exec();
}

void Ros1MasterChecker::checkMasterUp()
{
    //qDebug() << QThread::currentThreadId() << "check master up";
    static bool onceUp = false;
    bool isMasterUp = true;
    if (!onceUp)
        isMasterUp = ros::master::check();

    if (onceUp)
    {
        if (!ros::ok())
        {
            //All ros calls will fail now, we should move back to the original state
            debug(LOG_ERROR, "Ros1MasterChecker", "ROS not ok!");
            mNodeHandle.reset();
            mSpinner.reset();
            emit rosNotOk();
            quit(); //TODO - change to: throw exception, catch in main, return from main
        }
    }

    if (isMasterUp)
    {
        onceUp = true;
        if (mNodeHandle.isNull())
        {
            mNodeHandle.reset(new ros::NodeHandle());
        }
        if (mSpinner.isNull())
        {
            debug(LOG_WARNING, "Ros1MasterChecker", "Start spinning");
            mSpinner.reset(new ros::AsyncSpinner(4));
            mSpinner->start();
            debug(LOG_WARNING, "Ros1MasterChecker", "Past spinning");
        }
    }
    emit masterStatusUpdate(isMasterUp);
}

NodeHandlePtr Ros1MasterChecker::nodeHandle() const
{
    return mNodeHandle;
}
