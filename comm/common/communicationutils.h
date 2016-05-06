#ifndef COMMUNICATIONUTILS_H
#define COMMUNICATIONUTILS_H

#include <QString>
#include <QList>

namespace communication
{
    class CommunicationUtils
    {
    private:
        static quint8 randomByte();

    public:
        static QString randomString(int length = 12);
        static void dumpToHex(const int8_t *v, int size, QString context = "");

        template<typename T>
        static void fillRandomVector(int size, T& vector)
        {
            for (int i = 0; i < size; ++i)
            {
                vector.push_back(randomByte());
            }
        }
    };
}

#endif //COMMUNICATIONUTILS_H
