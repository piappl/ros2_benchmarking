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

QString CommunicationUtils::randomString(int length)
{
    const QString possibleCharacters("BENCHMARK");
    QString randomString;
    for(int i=0; i<length; ++i)
    {
       int index = qrand() % possibleCharacters.length();
       QChar nextChar = possibleCharacters.at(index);
       randomString.append(nextChar);
    }
    return randomString;
}

void CommunicationUtils::dumpToHex(const int8_t *v, int size, QString /*context*/)
{
    QByteArray array((const char *)v, size);
    QString hexString(array.toHex());
    debug(LOG_BENCHMARK, "PUBLISHING byte_msg", "id=0, content=%s", qPrintable(hexString));
}

