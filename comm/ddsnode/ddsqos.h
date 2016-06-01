#ifndef DDSQOS_H
#define DDSQOS_H

#include <common/qosprofiles.h>
#include <common/messagetypes.h>
#include "ddsinclude.h"

//TODO - this is just a stub. Implement it here (also for reader etc.)

namespace ddscommunication
{
    dds::pub::qos::DataWriterQos getQoS(communication::QoSSetting s);
}

#endif //DDSQOS_H
