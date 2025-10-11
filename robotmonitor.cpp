#include "robotmonitor.h"

RobotMonitor::RobotMonitor(QObject *parent) : QObject(parent)
{
    m_timer = new QTimer(this);
    // 500ms(0.5초)마다 checkRobotState 슬롯을 호출하도록 연결
    connect(m_timer, &QTimer::timeout, this, &RobotMonitor::checkRobotState);
}

void RobotMonitor::startMonitoring()
{
    qDebug() << "Robot monitoring thread started (500ms interval).";
    m_timer->start(500); // 500 밀리초마다 상태 확인
}

void RobotMonitor::checkRobotState()
{
    // 로봇 API를 통해 현재 로봇 상태를 가져옵니다.
    ROBOT_STATE currentState = GlobalDrfl.GetRobotState();

    // UI 업데이트를 위해 MainWindow로 시그널 전송
    emit robotStateChanged((int)currentState);
}
