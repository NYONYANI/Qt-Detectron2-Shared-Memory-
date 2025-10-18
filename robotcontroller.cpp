#include "robotcontroller.h"
#include <QDebug>
#include <QThread>
#include <QMatrix4x4>

RobotController::RobotController(QObject *parent) : QObject(parent)
{
    m_timer = new QTimer(this);
    connect(m_timer, &QTimer::timeout, this, &RobotController::checkRobotState);
}

void RobotController::startMonitoring()
{
    if (!m_timer->isActive()) {
        qDebug() << "[ROBOT_THREAD] Starting robot monitoring (100ms interval).";
        m_timer->start(100);
    }
}

void RobotController::checkRobotState()
{
    ROBOT_STATE currentState = GlobalDrfl.GetRobotState();
    emit robotStateChanged((int)currentState);

    LPROBOT_TASK_POSE pose_struct = GlobalDrfl.get_current_posx();
    float(*rotation_matrix)[3] = GlobalDrfl.get_current_rotm();

    if (pose_struct && rotation_matrix)
    {
        emit robotPoseUpdated(pose_struct->_fTargetPos);

        const float* pos = pose_struct->_fTargetPos;
        QMatrix4x4 transform(
            rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2], pos[0] / 1000.0f,
            rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2], pos[1] / 1000.0f,
            rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2], pos[2] / 1000.0f,
            0.0f,                  0.0f,                  0.0f,                  1.0f
            );
        emit robotTransformUpdated(transform);
    }
}

void RobotController::onMoveRobot(const QVector3D& position_mm, const QVector3D& orientation_deg)
{
    if (!g_bHasControlAuthority || GlobalDrfl.GetRobotState() != STATE_STANDBY) {
        qWarning() << "[ROBOT_THREAD] Cannot move: No control authority or not in STANDBY.";
        return;
    }
    qInfo() << "[ROBOT_THREAD] Received move request. Executing movel command.";
    float target_posx[6];
    target_posx[0] = position_mm.x();
    target_posx[1] = position_mm.y();
    target_posx[2] = position_mm.z();
    target_posx[3] = orientation_deg.x();
    target_posx[4] = orientation_deg.y();
    target_posx[5] = orientation_deg.z();

    float velx[2] = {100.0f, 60.0f};
    float accx[2] = {200.0f, 120.0f};

    qDebug() << "[ROBOT_THREAD] Moving to Pos(mm):" << target_posx[0] << target_posx[1] << target_posx[2]
             << "Rot(deg):" << target_posx[3] << target_posx[4] << target_posx[5];
    GlobalDrfl.movel(target_posx, velx, accx);
}

void RobotController::onRobotPickAndReturn(const QVector3D& target_pos_mm, const QVector3D& target_ori_deg,
                                           const QVector3D& approach_pos_mm, const QVector3D& approach_ori_deg)
{
    if (!g_bHasControlAuthority || GlobalDrfl.GetRobotState() != STATE_STANDBY) {
        qWarning() << "[ROBOT_THREAD] Cannot start sequence: No control authority or not in STANDBY.";
        return;
    }
    qInfo() << "[ROBOT_THREAD] ========== D Key: Pick Only (NO Return) ==========";

    float velx_down[2] = {80.0f, 40.0f};
    float accx_down[2] = {80.0f, 40.0f};

    // Step 1: 목표 위치로 하강
    qDebug() << "[ROBOT] Step 1/2: Descending to grasp position:" << target_pos_mm;
    float final_posx[6];
    final_posx[0] = target_pos_mm.x();
    final_posx[1] = target_pos_mm.y();
    final_posx[2] = target_pos_mm.z();
    final_posx[3] = target_ori_deg.x();
    final_posx[4] = target_ori_deg.y();
    final_posx[5] = target_ori_deg.z();

    if (GlobalDrfl.movel(final_posx, velx_down, accx_down)) {
        QThread::msleep(1500);

        // Step 2: 그리퍼 닫기
        qDebug() << "[ROBOT] Step 2/2: Closing gripper (Grasp)";
        onGripperAction(1);
        QThread::msleep(1500);

        qInfo() << "[ROBOT_THREAD] ========== Pick completed! Object grasped. Ready for MoveButton. ==========";
    } else {
        qWarning() << "[ROBOT_THREAD] ERROR: Move down to target failed.";
    }
}
void RobotController::onResetPosition()
{
    if (!g_bHasControlAuthority || GlobalDrfl.GetRobotState() != STATE_STANDBY) {
        qWarning() << "[ROBOT_THREAD] Cannot move: No control authority or not in STANDBY.";
        return;
    }
    qInfo() << "[ROBOT_THREAD] Moving to Reset Position.";
    float target_posx[6] = {349.0f, 9.21f, 378.46f, 172.0f, -139.0f, -179.0f};
    float velx[2] = {150.0f, 90.0f};
    float accx[2] = {300.0f, 180.0f};

    qDebug() << "[ROBOT_THREAD] Moving to Pos(mm):" << target_posx[0] << target_posx[1] << target_posx[2]
             << "Rot(deg):" << target_posx[3] << target_posx[4] << target_posx[5];
    GlobalDrfl.movel(target_posx, velx, accx);
}

void RobotController::onGripperAction(int action)
{
    if (!g_bHasControlAuthority || GlobalDrfl.GetRobotState() != STATE_STANDBY) {
        qWarning() << "[ROBOT_THREAD] Cannot perform action: No control authority or not in STANDBY.";
        return;
    }
    qDebug() << "[ROBOT_THREAD] Executing gripper: " << (action == 0 ? "OPEN" : "CLOSE");
    GlobalDrfl.set_tool_digital_output(GPIO_TOOL_DIGITAL_INDEX_1, false);
    GlobalDrfl.set_tool_digital_output(GPIO_TOOL_DIGITAL_INDEX_2, false);
    QThread::msleep(500);
    if (action == 0) {
        GlobalDrfl.set_tool_digital_output(GPIO_TOOL_DIGITAL_INDEX_1, true);
        GlobalDrfl.set_tool_digital_output(GPIO_TOOL_DIGITAL_INDEX_2, false);
        qDebug() << "[ROBOT_THREAD] Open command sent.";
    } else {
        GlobalDrfl.set_tool_digital_output(GPIO_TOOL_DIGITAL_INDEX_1, false);
        GlobalDrfl.set_tool_digital_output(GPIO_TOOL_DIGITAL_INDEX_2, true);
        qDebug() << "[ROBOT_THREAD] Close command sent.";
    }
    QThread::msleep(500);
}

// ✨ 헬퍼 함수
void RobotController::moveToPositionAndWait(const QVector3D& pos_mm, const QVector3D& ori_deg)
{
    if (!g_bHasControlAuthority || GlobalDrfl.GetRobotState() != STATE_STANDBY) {
        qWarning() << "[ROBOT_THREAD] Cannot move: No control authority or not in STANDBY.";
        return;
    }

    float target_posx[6];
    target_posx[0] = pos_mm.x(); target_posx[1] = pos_mm.y(); target_posx[2] = pos_mm.z();
    target_posx[3] = ori_deg.x(); target_posx[4] = ori_deg.y(); target_posx[5] = ori_deg.z();

    float velx[2] = {80.0f, 40.0f};
    float accx[2] = {80.0f, 40.0f};

    qDebug() << "[ROBOT] Moving to Pos:" << pos_mm << "Ori:" << ori_deg;

    if (!GlobalDrfl.movel(target_posx, velx, accx)) {
        qWarning() << "[ROBOT] Move command failed!";
    }
    // 참고: movel은 비동기일 수 있습니다.
    // 만약 movel이 즉시 리턴된다면, 이 함수 뒤에 QThread::msleep()이 필요합니다.
    // 기존 onLiftRotatePlaceSequence의 msleep(2000)을 보면 movel이 즉시 리턴되는 것 같습니다.
}

// ✨ 기존 Lift-Rotate-Place 시퀀스 (독립 실행용)
void RobotController::onLiftRotatePlaceSequence(const QVector3D& lift_pos_mm, const QVector3D& lift_ori_deg,
                                                const QVector3D& rotate_pos_mm, const QVector3D& rotate_ori_deg,
                                                const QVector3D& place_pos_mm, const QVector3D& place_ori_deg)
{
    if (!g_bHasControlAuthority || GlobalDrfl.GetRobotState() != STATE_STANDBY) {
        qWarning() << "[ROBOT] Cannot start sequence: No control authority or not in STANDBY.";
        return;
    }

    qDebug() << "[ROBOT] ========== Starting Lift-Rotate-Place Sequence (Manual) ==========";

    // Step 1: 3cm 위로 올라가기
    qDebug() << "[ROBOT] Step 1/5: Lifting 3cm up";
    moveToPositionAndWait(lift_pos_mm, lift_ori_deg);
    QThread::msleep(2000);

    // Step 2: 회전하기 (같은 높이에서)
    qDebug() << "[ROBOT] Step 2/5: Rotating to Y-aligned pose (Rz =" << rotate_ori_deg.z() << "deg)";
    moveToPositionAndWait(rotate_pos_mm, rotate_ori_deg);
    QThread::msleep(2000);

    // Step 3: 원래 높이로 내려가기
    qDebug() << "[ROBOT] Step 3/5: Placing back down to original height";
    moveToPositionAndWait(place_pos_mm, place_ori_deg);
    QThread::msleep(2000);

    // Step 4: 그리퍼 열기
    qDebug() << "[ROBOT] Step 4/5: Opening gripper (Release)";
    onGripperAction(0);
    QThread::msleep(2000);

    // Step 5: 다시 위로 올라가기
    qDebug() << "[ROBOT] Step 5/5: Lifting back up after release";
    moveToPositionAndWait(lift_pos_mm, place_ori_deg); // 수정된 방향 사용
    QThread::msleep(1000);

    qDebug() << "[ROBOT] ========== Lift-Rotate-Place Sequence Completed! ==========";
}


// ✨ [추가] Move 버튼을 위한 전체 자동 시퀀스
void RobotController::onFullPickAndPlaceSequence(
    const QVector3D& pre_grasp_pos_mm, const QVector3D& pre_grasp_ori_deg,
    const QVector3D& grasp_pos_mm, const QVector3D& grasp_ori_deg,
    const QVector3D& lift_pos_mm, const QVector3D& lift_ori_deg,
    const QVector3D& rotate_pos_mm, const QVector3D& rotate_ori_deg,
    const QVector3D& place_pos_mm, const QVector3D& place_ori_deg)
{
    if (!g_bHasControlAuthority || GlobalDrfl.GetRobotState() != STATE_STANDBY) {
        qWarning() << "[ROBOT_SEQ] Cannot start full sequence: No control authority or not in STANDBY.";
        return;
    }

    qDebug() << "[ROBOT_SEQ] ========== Starting Full Automated Sequence ==========";

    // Step 1: (M) 그리퍼 열기
    qDebug() << "[ROBOT_SEQ] Step 1/9: Opening Gripper (for M)";
    onGripperAction(0);
    QThread::msleep(1500); // 그리퍼 열릴 시간

    // Step 2: (M) Pre-Grasp 위치로 이동
    qDebug() << "[ROBOT_SEQ] Step 2/9: Moving to Pre-Grasp Pose (M)";
    moveToPositionAndWait(pre_grasp_pos_mm, pre_grasp_ori_deg);
    QThread::msleep(2000); // 이동 완료 대기

    // Step 3: (D) Grasp 위치로 하강
    qDebug() << "[ROBOT_SEQ] Step 3/9: Moving to Grasp Pose (D-part 1)";
    moveToPositionAndWait(grasp_pos_mm, grasp_ori_deg);
    QThread::msleep(2000); // 이동 완료 대기

    // Step 4: (D) 그리퍼 닫기
    qDebug() << "[ROBOT_SEQ] Step 4/9: Closing Gripper (D-part 2)";
    onGripperAction(1);
    QThread::msleep(1500); // 그리퍼 닫힐 시간

    // Step 5: (Move) 3cm 위로 올라가기
    qDebug() << "[ROBOT_SEQ] Step 5/9: Lifting object (Move-part 1)";
    moveToPositionAndWait(lift_pos_mm, lift_ori_deg);
    QThread::msleep(2000); // 이동 완료 대기

    // Step 6: (Move) 회전하기 (같은 높이에서)
    qDebug() << "[ROBOT_SEQ] Step 6/9: Rotating object (Move-part 2)";
    moveToPositionAndWait(rotate_pos_mm, rotate_ori_deg);
    QThread::msleep(2000); // 이동 완료 대기

    // Step 7: (Move) 원래 높이로 내려가기
    qDebug() << "[ROBOT_SEQ] Step 7/9: Placing object (Move-part 3)";
    moveToPositionAndWait(place_pos_mm, place_ori_deg);
    QThread::msleep(2000); // 이동 완료 대기

    // Step 8: (Move) 그리퍼 열기
    qDebug() << "[ROBOT_SEQ] Step 8/9: Opening gripper (Move-part 4)";
    onGripperAction(0);
    QThread::msleep(1500); // 그리퍼 열릴 시간

    // Step 9: (Move) 다시 위로 올라가기
    qDebug() << "[ROBOT_SEQ] Step 9/9: Lifting back up (Move-part 5)";
    moveToPositionAndWait(rotate_pos_mm, place_ori_deg); // 최종 방향(place_ori_deg) 유지
    QThread::msleep(1000); // 이동 완료 대기

    qDebug() << "[ROBOT_SEQ] ========== Full Automated Sequence Completed! ==========";
}
