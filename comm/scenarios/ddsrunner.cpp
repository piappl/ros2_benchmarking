#include "nodestarter.h"
#include <ddsnode/ddsnodefactory.h>

int main(int argc, char *argv[])
{
    NodeFactoryInterfacePtr factory(new DDSNodeFactory());
    return nodestarter::startNode(argc, argv, factory);
}
