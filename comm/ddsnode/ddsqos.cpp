#include "ddsqos.h"
#include "common/qosprofiles.h"
#include "common/logging.h"

using namespace communication;
using namespace dds::core;

namespace ddscommunication
{
    dds::pub::qos::DataWriterQos getWriterQoS(communication::QoSSetting s)
    {
        dds::pub::qos::DataWriterQos qos;
        switch (s.profile)
        {
            case QoSProfileAlarm:
                debug(LOG_WARNING, "DataWriterQos", "Setting QosProfileAlarm");
                qos << policy::Reliability::Reliable();
                qos << policy::Durability::TransientLocal();
                qos << policy::History::KeepAll();
                break;
            case QoSProfileControl:
                debug(LOG_WARNING, "DataWriterQos", "Setting QosProfileControl");
                qos << policy::Reliability::Reliable();
                qos << policy::Durability::Volatile();
                qos << policy::History::KeepLast(10);
                break;
            case QoSProfileStatus:
            case QoSProfileSensor:
                debug(LOG_WARNING, "DataWriterQos", "Setting QosProfileStatus|Sensor");
                qos << policy::Reliability::BestEffort();
                qos << policy::Durability::Volatile();
                break;
            default:
                debug(LOG_WARNING, "DataWriterQos", "Setting default QoS profile");
                break;
        }
        return qos;
    }

    dds::sub::qos::DataReaderQos getReaderQoS(communication::QoSSetting s)
    {
        dds::sub::qos::DataReaderQos qos;
        switch (s.profile)
        {
            case QoSProfileAlarm:
                debug(LOG_WARNING, "DataReaderQos", "Setting QosProfileAlarm");
                qos << policy::Reliability::Reliable();
                qos << policy::Durability::TransientLocal();
                qos << policy::History::KeepAll();
                break;
            case QoSProfileControl:
                debug(LOG_WARNING, "DataReaderQos", "Setting QosProfileControl");
                qos << policy::Reliability::Reliable();
                qos << policy::Durability::Volatile();
                qos << policy::History::KeepLast(10);
                break;
            case QoSProfileStatus:
            case QoSProfileSensor:
                debug(LOG_WARNING, "DataReaderQos", "Setting QosProfileStatus|Sensor");
                qos << policy::Reliability::BestEffort();
                qos << policy::Durability::Volatile();
                break;
            default:
                debug(LOG_WARNING, "DataReaderQos", "Setting default QoS profile");
                break;
        }
        return qos;
    }

}
