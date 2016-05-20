#ifndef DDSSUBSCRIBER_H
#define DDSSUBSCRIBER_H

#include <common/messagetypes.h>
#include "ddsinclude.h"
#include "ddstopics.h"

namespace ddscommunication
{   //TODO - refactor
    class CmdVelListener : public dds::sub::NoOpDataReaderListener<MoveBaseDDSType>
    {
    public:
        virtual void on_data_available(dds::sub::DataReader<MoveBaseDDSType>& dr);
        virtual void on_liveliness_changed(dds::sub::DataReader<MoveBaseDDSType>& dr,
                  const dds::core::status::LivelinessChangedStatus& status);
    };

    class RobotControlListener : public dds::sub::NoOpDataReaderListener<RobotControlDDSType>
    {
    public:
        virtual void on_data_available(dds::sub::DataReader<RobotControlDDSType>& dr);
        virtual void on_liveliness_changed(dds::sub::DataReader<RobotControlDDSType>& dr,
                  const dds::core::status::LivelinessChangedStatus& status);
    };

    class RobotStatusListener : public dds::sub::NoOpDataReaderListener<RobotStatusDDSType>
    {
    public:
        virtual void on_data_available(dds::sub::DataReader<RobotStatusDDSType>& dr);
        virtual void on_liveliness_changed(dds::sub::DataReader<RobotStatusDDSType>& dr,
                  const dds::core::status::LivelinessChangedStatus& status);
    };

    class BytesListener : public dds::sub::NoOpDataReaderListener<BytesDDSType>
    {
    public:
        virtual void on_data_available(dds::sub::DataReader<BytesDDSType>& dr);
        virtual void on_liveliness_changed(dds::sub::DataReader<BytesDDSType>& dr,
                  const dds::core::status::LivelinessChangedStatus& status);
    };


    class DDSSubscriber
    {
        public:
            DDSSubscriber(const Participant& participant, const DDSTopics &topics);
            void subscribe(communication::MessageType t);
            void unsubscribe(communication::MessageType t);
        private:
            Subscriber mSubsciber;

            //TODO - generalize (?)
            Reader<MoveBaseDDSType> mCmdVelReader;
            Reader<BytesDDSType> mBytesReader;
            Reader<RobotControlDDSType> mControlReader;
            Reader<RobotStatusDDSType> mStatusReader;

            CmdVelListener mCmdVelListener;
            RobotControlListener mControlListener;
            RobotStatusListener mStatusListener;
            BytesListener mBytesListener;
    };
}
#endif //DDSSUBSCRIBER_H
