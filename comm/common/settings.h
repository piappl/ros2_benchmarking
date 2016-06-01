#ifndef SETTINGS_H
#define SETTINGS_H

#include <common/qosprofiles.h>

namespace communication
{
    struct Settings
    {   //TODO - this should evolve to QVariant map or QSetting container, suitable for now
        QString nodeName;
        int domainID;
        QoSSettings qos;
    };
}

#endif //SETTINGS_H
