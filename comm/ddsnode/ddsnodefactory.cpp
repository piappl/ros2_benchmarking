#include "ddsnodefactory.h"
#include "ddsnode.h"
#include <common/logging.h>

using namespace communication;

NodeInterfacePtr DDSNodeFactory::makeNode(communication::Settings s)
{
    int id = s.domainID;
    if (id != 0)
    {
        debug(LOG_ERROR, "DDSNodeFactory", "Only default domain id supported for now (id=0)");
        exit(1);
    }
    return NodeInterfacePtr(new DDSNode(s));
}
