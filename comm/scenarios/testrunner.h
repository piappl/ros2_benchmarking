#include <QObject>
#include <QTimer>

#include <common/rosnodeinterface.h>
#include "configparser.h"

class TestRunner : public QObject
{
    Q_OBJECT
public:
    TestRunner(QString configFile, roscommunication::RosNodeInterfacePtr node);

signals:
    void quit();

private slots:
    void publishCmdVel();
    void publishRobotControl();
    void publishRobotStatus();
    void publishBytes();

    void startTest();
    void finishTest();

private:
    void initTimers();
    void advertise();
    void subscribe(bool subsribe = true);

    roscommunication::RosNodeInterfacePtr mNode;
    ConfigParser mConfig;

    QTimer mCmdVelTimer;
    QTimer mRobotControlTimer;
    QTimer mRobotStatusTimer;
    QTimer mBytesTimer;
    QTimer mStartDelayTimer;
    QTimer mTestTimer;
    QTimer mQuitDelayTimer;
};
