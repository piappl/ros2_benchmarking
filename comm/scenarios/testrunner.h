#include <QObject>
#include <QTimer>

#include <common/nodeinterface.h>
#include "configparser.h"

class TestRunner : public QObject
{
    Q_OBJECT
public:
    TestRunner(ConfigParser *p, communication::NodeInterfacePtr node);

signals:
    void quit();

private slots:
    void publishRobotControl();
    void publishRobotAlarm();
    void publishRobotSensor();

    void startTest();
    void finishTest();

private:
    void initTimers();
    void advertise();
    void subscribe();
    void unsubscribe();

    ConfigParser *mConfig;
    communication::NodeInterfacePtr mNode;

    QTimer mRobotAlarmTimer;
    QTimer mRobotControlTimer;
    QTimer mRobotSensorTimer;
    QTimer mStartDelayTimer;
    QTimer mTestTimer;
    QTimer mQuitDelayTimer;
};
