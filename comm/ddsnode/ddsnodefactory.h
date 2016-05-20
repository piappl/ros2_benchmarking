#ifndef DDSNODEFACTORY_H
#define DDSNODEFACTORY_H

#include <common/nodefactoryinterface.h>

class DDSNodeFactory : public NodeFactoryInterface
{
public:
    communication::NodeInterfacePtr makeNode(QString idString);
};
typedef QSharedPointer<DDSNodeFactory> DDSNodeFactoryPtr;

#endif //DDSNODEFACTORY_H
