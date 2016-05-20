#ifndef NODESTARTER_H
#define NODESTARTER_H

#include <common/nodefactoryinterface.h>

namespace nodestarter
{
    int startNode(int argc, char *argv[], NodeFactoryInterfacePtr factory);
}

#endif //NODESTARTER_H
