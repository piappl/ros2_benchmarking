#include <QByteArray>
#include <common/logging.h>
#include <common/communicationutils.h>

using namespace communication;

quint8 CommunicationUtils::randomByte()
{
    const uint seed = 1337; //Want the seed to be the same every time
    static bool initializeRand = true;
    if (initializeRand)
    {
        qsrand(seed);
        initializeRand = false;
    }

    return static_cast<quint8>((qrand() % 256)); //0-255
}

std::string CommunicationUtils::randomString(int length)
{
    static const char alphanum[] =
            "0123456789"
            "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
            "abcdefghijklmnopqrstuvwxyz";
    std::string string(length, '_');
    for(int i = 0; i < length; ++i)
    {
       string[i] = alphanum[qrand() % (sizeof(alphanum) - 1)];
    }
    return string;
}
