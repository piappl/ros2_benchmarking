#include "nodestarter.h"
#include <ros1node/ros1nodefactory.h>

int main(int argc, char *argv[])
{
    NodeFactoryInterfacePtr factory(new Ros1NodeFactory());
    return nodestarter::startNode(argc, argv, factory);
}
