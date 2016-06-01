#include "ddsqos.h"

namespace ddscommunication
{
    dds::pub::qos::DataWriterQos getQoS(communication::QoSSetting /*s*/)
    {
        //IMPLEMENT.
        return dds::pub::qos::DataWriterQos(); //DATAWRITER_QOS_DEFAULT ?
    }
}
