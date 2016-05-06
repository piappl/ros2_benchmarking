#include "ros1participant.h"
#include <common/logging.h>

using namespace roscommunication;

Ros1Participant::Ros1Participant(NodeHandlePtr handle)
    : mNodeHandle(handle), mMasterUp(false)
{
    //debug(LOG_WARNING, "Ros1Participant", "Creation");
}

Ros1Participant::~Ros1Participant()
{
    //debug(LOG_WARNING, "Ros1Participant", "Destruction");
}

bool Ros1Participant::isNodeOperating() const
{
    if (!mNodeHandle)
        return false;

    if (!mMasterUp)
        return false;

    return true;
}

void Ros1Participant::setNodeHandle(NodeHandlePtr nodeHandle)
{
    mNodeHandle = nodeHandle;
}

void Ros1Participant::setMasterUp(bool up)
{
    if (!up && mMasterUp)
    {
        stop();
    }

    mMasterUp = up;
    if (isNodeOperating())
    {
        synchronize();
    }
}

NodeHandlePtr Ros1Participant::nodeHandle() const
{
    return mNodeHandle;
}
