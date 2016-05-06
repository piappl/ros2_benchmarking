#include <QtCore/QCoreApplication>
#include <QFileInfo>
#include <common/logging.h>
#include <ros1node/ros1node.h>
#include "testrunner.h"

using namespace roscommunication;

//TODO - common code in mains (ros1, ros2) to refactor

int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        debug(LOG_ERROR, "MAIN", "Usage: %s <rosnodename> <configfilename>. \
              Ros node names must differ from each other in running nodes", argv[0]);
        return -1;
    }

    QCoreApplication a(argc, argv);
    QString rosnodeName(argv[1]);
    QString configFile(argv[2]);

    QFileInfo check_file(configFile);
        // check if file exists and if yes: Is it really a file and no directory?
    if (!check_file.exists() || !check_file.isFile())
    {
        debug(LOG_ERROR, "MAIN", "Config file %s does not exist!", qPrintable(configFile));
        return -1;
    }

    debug(LOG_BENCHMARK, "MAIN", "Initializing test of ROS1 with node name %s and config %s",
          qPrintable(rosnodeName), qPrintable(configFile));

    RosNodeInterfacePtr node(new Ros1Node(rosnodeName));
    TestRunner runner(configFile, node);
    QObject::connect(&runner, SIGNAL(quit()), &a, SLOT(quit()));

    a.exec();
    debug(LOG_BENCHMARK, "MAIN", "Exiting");
}
