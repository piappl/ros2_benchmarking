#ifndef ROS1NODEFACTORY_H
#define ROS1NODEFACTORY_H

#include <common/nodefactoryinterface.h>
#include <ros1node/ros1node.h>

//TODO - deps to cpp
class Ros1NodeFactory : public NodeFactoryInterface
{
public:
    communication::NodeInterfacePtr makeNode(communication::Settings s)
    {
        return communication::NodeInterfacePtr(new roscommunication::Ros1Node(s));
    }
};
typedef QSharedPointer<Ros1NodeFactory> Ros1NodeFactoryPtr;

#endif //ROS1NODEFACTORY_H
