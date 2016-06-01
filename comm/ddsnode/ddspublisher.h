#ifndef DDSPUBLISHER_H
#define DDSPUBLISHER_H

#include <common/messagetypes.h>
#include <common/qosprofiles.h>
#include "ddsinclude.h"
#include "ddstopics.h"

namespace ddscommunication
{
    class DDSPublisher
    {
        public:
            DDSPublisher(const Participant& participant,
                         const DDSTopics &topics, communication::QoSSettings qos);
            void advertise(communication::MessageType type);
            void publishCmdVel(communication::MoveBase);
            void publishRobotStatus(communication::RobotStatus status);
            void publishRobotControl(communication::RobotControl control);
            void publishByteMessage(int size);

        private:
            Publisher mPublisher;

            //TODO - generalize?
            Writer<MoveBaseDDSType> mCmdVelWriter;
            Writer<BytesDDSType> mBytesWriter;
            Writer<RobotControlDDSType> mControlWriter;
            Writer<RobotStatusDDSType> mStatusWriter;
    };
}
#endif //DDSPUBLISHER_H
