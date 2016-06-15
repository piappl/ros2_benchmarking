#ifndef QOSPROFILES_H
#define QOSPROFILES_H

#include <QMap>
#include <common/messagetypes.h>

namespace communication
{
    enum QoSProfile
    {
        QoSProfileDefault,
        QoSProfileSensor,
        QoSProfileAlarm,
        QoSProfileControl
    };

    struct QoSSetting
    {   //TODO - struct bc qos settings can be separate for: subscribers, publishers, writers, readers.
        QoSProfile profile;
    };

    typedef QMap<communication::MessageType, QoSSetting> QoSSettings;
}

#endif //QOSPROFILES_H
