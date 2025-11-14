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

    // 위치와 오일러 각도 값을 포함한 구조체 가져오기
    LPROBOT_TASK_POSE pose_struct = GlobalDrfl.get_current_posx();

    // ✨ [추가] 3x3 회전 행렬 직접 가져오기
    float(*rotation_matrix)[3] = GlobalDrfl.get_current_rotm();

    if (pose_struct && rotation_matrix)
    {
        // 기존 UI 라벨 업데이트를 위해 pose 배열 시그널 전송 (유지)
        emit robotPoseUpdated(pose_struct->_fTargetPos);

        // ✨ [추가] Python과 동일하게 위치 벡터와 회전 행렬로 4x4 변환 행렬 생성
        const float* pos = pose_struct->_fTargetPos; // X, Y, Z 위치
        QMatrix4x4 transform(
            rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2], pos[0] / 1000.0f,
            rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2], pos[1] / 1000.0f,
            rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2], pos[2] / 1000.0f,
            0.0f,                  0.0f,                  0.0f,                  1.0f
            );

        // ✨ [추가] 생성된 변환 행렬을 새로운 시그널로 전송
        emit robotTransformUpdated(transform);
    }
}
