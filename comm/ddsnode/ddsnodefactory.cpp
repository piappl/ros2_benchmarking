#include "ddsnodefactory.h"
#include "ddsnode.h"
#include <common/logging.h>

using namespace communication;

NodeInterfacePtr DDSNodeFactory::makeNode(QString idString)
{
    bool ok;
    int id = idString.toInt(&ok);
    if (!ok)
    {
        debug(LOG_ERROR, "DDSNodeFactory", "Invalid ID - must be an integer!");
        exit(1);
    }
    return NodeInterfacePtr(new ddscommunication::DDSNode(id));
}
