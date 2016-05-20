#ifndef ROS1NODEFACTORY_H
#define ROS1NODEFACTORY_H

#include <common/nodefactoryinterface.h>
#include <ros1node/ros1node.h>

//TODO - deps to cpp
class Ros1NodeFactory : public NodeFactoryInterface
{
public:
    communication::NodeInterfacePtr makeNode(QString nodeName)
    {
        return communication::NodeInterfacePtr(new roscommunication::Ros1Node(nodeName));
    }
};
typedef QSharedPointer<Ros1NodeFactory> Ros1NodeFactoryPtr;

#endif //ROS1NODEFACTORY_H
