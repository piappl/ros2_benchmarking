#include <ros1node/ros1node.h>
#include <QObject>
#include <QTimer>

class Ros1Console : public QObject
{
    Q_OBJECT
public:
    Ros1Console();

signals:
    void quit();

private slots:
    void publishCV();
    void publishRC();
    void finishTest();

private:
    roscommunication::Ros1Node mNode;
    QTimer mCVTimer;
    QTimer mRCTimer;
    QTimer mWholeTestTimer;
    QTimer mQuitDelayTimer;
};
