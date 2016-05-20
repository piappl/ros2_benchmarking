#ifndef NODEFACTORYINTERFACE_H
#define NODEFACTORYINTERFACE_H

#include <common/nodeinterface.h>

//TODO - node factory might not be needed, if makeNode impl. are trivials and deps on includes with ctor
//for nodes are acceptable for library users

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
