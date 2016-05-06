#include "ros1initializer.h"
#include <ros/ros.h>
#include <common/logging.h>

using namespace roscommunication;

Ros1Initializer::Ros1Initializer(QString name)
    : mInitialized(false)
{
    ros::VP_string initString;
    QString nodeName = name;
    ros::init(initString, nodeName.toStdString());

    if(!ros::isInitialized())
    {
        debug(LOG_ERROR, "RosInitializer", "Could not initialize ROS");
        return;
    }
    mInitialized = true;
    debug(LOG_WARNING, "RosInitializer", "Initialized");
    ROS_INFO("ROS initialized");
}

bool Ros1Initializer::initialized() const
{
    return mInitialized;
}
