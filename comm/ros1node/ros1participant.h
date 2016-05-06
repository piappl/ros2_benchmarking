#ifndef ROS1PARTICIPANT_H
#define ROS1PARTICIPANT_H

#include <QString>
#include "ros1fwd.h"

namespace roscommunication
{
    static const int kQueueSize = 100;
    class Ros1Participant
    {
    public:
        virtual ~Ros1Participant();
        void setMasterUp(bool up);
        void setNodeHandle(NodeHandlePtr nodeHandle);

    protected:
        Ros1Participant(NodeHandlePtr handle);
        bool isNodeOperating() const;
        NodeHandlePtr nodeHandle() const;

    private:
        virtual void stop() {}
        virtual void synchronize() = 0;

        NodeHandlePtr mNodeHandle;
        bool mMasterUp;
    };
}

#endif // ROS1PARTICIPANT_H
