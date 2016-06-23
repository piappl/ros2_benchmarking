#include "ddsqos.h"
#include "common/qosprofiles.h"
#include "common/logging.h"

using namespace communication;
using namespace dds::core;

namespace communication
{
    dds::pub::qos::DataWriterQos getWriterQoS(communication::QoSSetting s)
    {
        return getQoS<dds::pub::qos::DataWriterQos>(s);
    }

    dds::sub::qos::DataReaderQos getReaderQoS(communication::QoSSetting s)
    {
        return getQoS<dds::sub::qos::DataReaderQos>(s);
    }

    template <class T>
    T getQoS(communication::QoSSetting s)
    {
        T qos;
        qos << policy::ResourceLimits(dds::core::LENGTH_UNLIMITED, dds::core::LENGTH_UNLIMITED, dds::core::LENGTH_UNLIMITED);
        //qos << policy::Liveliness::ManualByTopic(Duration::from_secs(10));
        switch (s.profile)
        {
            case QoSProfileAlarm:
                debug(LOG_BENCHMARK, "QoS", "Setting QosProfileAlarm");
                qos << policy::Reliability::Reliable();
                qos << policy::Durability::TransientLocal();
                qos << policy::History::KeepAll();
                break;
            case QoSProfileControl:
                debug(LOG_BENCHMARK, "QoS", "Setting QosProfileControl");
                qos << policy::Reliability::Reliable();
                qos << policy::Durability::Volatile();
                qos << policy::History::KeepLast(10);
                break;
            case QoSProfileSensor:
                debug(LOG_BENCHMARK, "QoS", "Setting QosProfileSensor");
                qos << policy::Reliability::BestEffort();
                qos << policy::Durability::Volatile();
                break;
            default:
                debug(LOG_WARNING, "QoS", "Setting default QoS profile");
                break;
        }
        return qos;
    }

}
