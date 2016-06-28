#include <QtCore/QCoreApplication>
#include <QFileInfo>
#include <common/logging.h>
#include "testrunner.h"
#include "nodestarter.h"
#include "configparser.h"

//Note - move to a simpler version with template instead of factory if nodes ctor deps acceptable
//in runners: startNode<T>, make_shared
namespace nodestarter
{
    int startNode(int argc, char *argv[], NodeFactoryInterfacePtr factory)
    {
        if (argc != 2)
        {
            debug(LOG_ERROR, "startNode", "Usage: %s <configfilename>", argv[0]);
            return -1;
        }

        QCoreApplication a(argc, argv);
        QString configFile(argv[1]);

        QFileInfo check_file(configFile);
        // check if file exists and if yes: Is it really a file and no directory?
        if (!check_file.exists() || !check_file.isFile())
        {
            debug(LOG_ERROR, "startNode", "Config file %s does not exist!", qPrintable(configFile));
            return -1;
        }

        debug(LOG_BENCHMARK, "startNode", "Initializing test of %s with config %s",
              argv[0], qPrintable(configFile));

        ConfigParser p(configFile);
        communication::NodeInterfacePtr node = factory->makeNode(p.nodeConfig());
        TestRunner runner(&p, node);
        QObject::connect(&runner, SIGNAL(quit()), &a, SLOT(quit()));

        auto retval = a.exec();
        debug(LOG_BENCHMARK, "startNode", "Exiting");
        return retval;
    }
}
