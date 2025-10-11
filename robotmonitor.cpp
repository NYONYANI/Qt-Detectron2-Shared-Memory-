#include "robotmonitor.h"

RobotMonitor::RobotMonitor(QObject *parent) : QObject(parent)
{
    m_timer = new QTimer(this);
    connect(m_timer, &QTimer::timeout, this, &RobotMonitor::checkRobotState);
}

void RobotMonitor::startMonitoring()
{
    qDebug() << "Robot monitoring thread started (100ms interval).";
    m_timer->start(100);
}

void RobotMonitor::checkRobotState()
{
    ROBOT_STATE currentState = GlobalDrfl.GetRobotState();
    emit robotStateChanged((int)currentState);

    // ✨ get_current_posx()를 인수 없이 호출하여 LPROBOT_TASK_POSE를 받습니다.
    LPROBOT_TASK_POSE pose_struct = GlobalDrfl.get_current_posx();

    if (pose_struct)
    {
        // ✨ 6개 좌표 값(X,Y,Z,Rx,Ry,Rz)이 담긴 _fTargetPos 배열을 시그널로 전송합니다.
        emit robotPoseUpdated(pose_struct->_fTargetPos);
    }
}
