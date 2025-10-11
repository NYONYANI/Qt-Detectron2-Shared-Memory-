#ifndef ROBOTMONITOR_H
#define ROBOTMONITOR_H

#include <QObject>
#include <QTimer>
#include <QDebug>
#include "DRFLEx.h"
using namespace DRAFramework;
// 💡 GlobalDrfl 객체 사용을 위해 extern으로 선언합니다.
extern CDRFLEx GlobalDrfl;

class RobotMonitor : public QObject
{
    Q_OBJECT

public:
    explicit RobotMonitor(QObject *parent = nullptr);

signals:
    // 로봇 상태가 변경되었음을 MainWindow에 int 값으로 알리는 시그널
    void robotStateChanged(int state);

public slots:
    // 스레드 시작 시 호출될 메인 함수
    void startMonitoring();

private slots:
    // 주기적인 상태 확인 작업을 수행할 슬롯 (500ms 주기)
    void checkRobotState();

private:
    QTimer *m_timer;
};

#endif // ROBOTMONITOR_H
