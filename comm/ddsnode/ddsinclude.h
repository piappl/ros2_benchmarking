#ifndef DDSINCLUDE_H
#define DDSINCLUDE_H

#include <QSharedPointer>
#include "gen/RobotMessages_DCPS.hpp"

namespace communication
{
    typedef dds::domain::DomainParticipant Participant;

    typedef dds::pub::Publisher Publisher;
    typedef QSharedPointer<Publisher> PublisherPtr;

    typedef dds::sub::Subscriber Subscriber;
    typedef QSharedPointer<Subscriber> SubscriberPtr;

    template<typename T>
    using Writer = dds::pub::DataWriter<T>;

    template<typename T>
    using WriterPtr = QSharedPointer<dds::pub::DataWriter<T> >;

    template<typename T>
    using Reader = dds::sub::DataReader<T>;

    template<typename T>
    using ReaderPtr = QSharedPointer<dds::sub::DataReader<T> >;

    template<typename T>
    using Topic = dds::topic::Topic<T>;

    const int kDomainID = 1;
}

#endif //DDSINCLUDE_H
