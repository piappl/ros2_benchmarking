#ifndef DDSNODEFACTORY_H
#define DDSNODEFACTORY_H

#include <common/nodefactoryinterface.h>

class DDSNodeFactory : public NodeFactoryInterface
{
public:
    communication::NodeInterfacePtr makeNode(communication::Settings s);
};
typedef QSharedPointer<DDSNodeFactory> DDSNodeFactoryPtr;

#endif //DDSNODEFACTORY_H
