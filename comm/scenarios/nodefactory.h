#ifndef NODEFACTORYINTERFACE_H
#define NODEFACTORYINTERFACE_H

#include <common/nodeinterface.h>

class NodeFactoryInterface
{
public:
    enum NodeType
    {
        NodeTypeRos1,
        NodeTypeRos2,
        NodeTypeDDS
    };

    virtual communication::NodeInterfacePtr makeNode(QString nodeName) = 0;
};
typedef QSharedPointer<NodeFactoryInterface> NodeFactoryInterfacePtr;

#endif //NODEFACTORYINTERFACE_H
