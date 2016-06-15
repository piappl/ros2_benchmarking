#ifndef DDSQOS_H
#define DDSQOS_H

#include <common/qosprofiles.h>
#include <common/messagetypes.h>
#include "ddsinclude.h"

namespace communication
{
    dds::pub::qos::DataWriterQos getWriterQoS(communication::QoSSetting s);
    dds::sub::qos::DataReaderQos getReaderQoS(communication::QoSSetting s);
    template <class T>
    T getQoS(communication::QoSSetting s);
}

#endif //DDSQOS_H
