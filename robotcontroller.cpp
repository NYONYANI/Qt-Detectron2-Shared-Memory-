#include "robotcontroller.h"
#include <QDebug>
#include <QThread>
#include <QMatrix4x4>
#include <QtMath> // qRadiansToDegrees 사용
#include <cstring> // memcpy 사용

RobotController::RobotController(QObject *parent)
    : QObject(parent)
    , m_angleDebugPrinted(false)
{
    m_timer = new QTimer(this);
    connect(m_timer, &QTimer::timeout, this, &RobotController::checkRobotState);
}

void RobotController::startMonitoring()
{
    if (!m_timer->isActive()) {
        qDebug() << "[ROBOT_THREAD] Starting robot monitoring (100ms interval).";
        m_angleDebugPrinted = false;
        m_timer->start(100);
    }
}

void RobotController::checkRobotState()
{
    ROBOT_STATE currentState = GlobalDrfl.GetRobotState();
    emit robotStateChanged((int)currentState);

    LPROBOT_TASK_POSE pose_struct = GlobalDrfl.get_current_posx();
    float(*rotation_matrix_ptr)[3] = GlobalDrfl.get_current_rotm();

    if (pose_struct && rotation_matrix_ptr)
    {
        float rotation_matrix[3][3];
        memcpy(rotation_matrix, rotation_matrix_ptr, sizeof(rotation_matrix));

        if (!m_angleDebugPrinted) {
            float gui_a = pose_struct->_fTargetPos[3];
            float gui_b = pose_struct->_fTargetPos[4];
            float gui_c = pose_struct->_fTargetPos[5];
            qDebug() << "[Angle Debug] GUI (A, B, C):" << gui_a << gui_b << gui_c;

            QMatrix3x3 rotMat(rotation_matrix[0]);

            QQuaternion quat = QQuaternion::fromRotationMatrix(rotMat);

            QVector3D eulerXYZ = rotationMatrixToEulerAngles(rotMat, "XYZ");
            qDebug() << "[Angle Debug] Euler XYZ (Roll, Pitch, Yaw):" << eulerXYZ;

            QVector3D eulerZYX = rotationMatrixToEulerAngles(rotMat, "ZYX");
            qDebug() << "[Angle Debug] Euler ZYX (Yaw, Pitch, Roll):" << eulerZYX;

            QVector3D eulerZYZ = rotationMatrixToEulerAngles(rotMat, "ZYZ");
            qDebug() << "[Angle Debug] Euler ZYZ:" << eulerZYZ;

            QVector3D eulerXZY = rotationMatrixToEulerAngles(rotMat, "XZY");
            qDebug() << "[Angle Debug] Euler XZY:" << eulerXZY;

            QVector3D eulerYXZ = rotationMatrixToEulerAngles(rotMat, "YXZ");
            qDebug() << "[Angle Debug] Euler YXZ:" << eulerYXZ;

            QVector3D eulerYZX = rotationMatrixToEulerAngles(rotMat, "YZX");
            qDebug() << "[Angle Debug] Euler YZX:" << eulerYZX;
            qDebug() << "---------------------------------------------";

            m_angleDebugPrinted = true;
        }

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


QVector3D RobotController::rotationMatrixToEulerAngles(const QMatrix3x3& R, const QString& order)
{
    float r11 = R(0,0), r12 = R(0,1), r13 = R(0,2);
    float r21 = R(1,0), r22 = R(1,1), r23 = R(1,2);
    float r31 = R(2,0), r32 = R(2,1), r33 = R(2,2);
    float x=0, y=0, z=0;

    if (order == "XYZ") {
        y = asin(qBound(-1.0f, r13, 1.0f));
        if (qAbs(qCos(y)) > 1e-6) {
            x = atan2(-r23, r33);
            z = atan2(-r12, r11);
        } else {
            x = atan2(r32, r22);
            z = 0;
        }
    } else if (order == "ZYX") {
        y = asin(-r31);
        if (qAbs(qCos(y)) > 1e-6) {
            x = atan2(r32, r33);
            z = atan2(r21, r11);
        } else {
            x = atan2(-r23, r22);
            z = 0;
        }
    } else if (order == "ZYZ") {
        y = acos(qBound(-1.0f, r33, 1.0f));
        if (qAbs(sin(y)) > 1e-6) {
            x = atan2(r23, r13);
            z = atan2(r32, -r31);
        } else {
            x = atan2(-r12, r22);
            z = 0;
        }
    } else if (order == "XZY") {
        z = asin(-qBound(-1.0f, r12, 1.0f));
        if (qAbs(qCos(z)) > 1e-6) {
            x = atan2(r32, r22);
            y = atan2(r13, r11);
        } else {
            x = atan2(-r23, r33);
            y = 0;
        }
    } else if (order == "YXZ") {
        x = asin(qBound(-1.0f, r23, 1.0f));
        if (qAbs(qCos(x)) > 1e-6) {
            y = atan2(-r13, r33);
            z = atan2(-r21, r22);
        } else {
            y = atan2(r31, r11);
            z = 0;
        }
    } else if (order == "YZX") {
        z = asin(qBound(-1.0f, r21, 1.0f));
        if (qAbs(qCos(z)) > 1e-6) {
            x = atan2(-r23, r22);
            y = atan2(-r31, r11);
        } else {
            x = 0;
            y = atan2(r13, r33);
        }
    }
    else {
        qWarning() << "[Angle Debug] Unsupported Euler order:" << order;
        QQuaternion quat = QQuaternion::fromRotationMatrix(R);
        return quat.toEulerAngles();
    }

    return QVector3D(qRadiansToDegrees(x), qRadiansToDegrees(y), qRadiansToDegrees(z));
}


void RobotController::onMoveRobot(const QVector3D& position_mm, const QVector3D& orientation_deg)
{
    GlobalDrfl.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
    GlobalDrfl.set_robot_system(ROBOT_SYSTEM_REAL);

    if (!g_bHasControlAuthority || GlobalDrfl.GetRobotState() != STATE_STANDBY) {
        qWarning() << "[ROBOT_THREAD] Cannot move: No control authority or not in STANDBY.";
        return;
    }
    qInfo() << "[ROBOT_THREAD] Received move request. Checking IK...";

    float target_posx[6];
    target_posx[0] = position_mm.x();
    target_posx[1] = position_mm.y();
    target_posx[2] = position_mm.z();
    target_posx[3] = orientation_deg.x();
    target_posx[4] = orientation_deg.y();
    target_posx[5] = orientation_deg.z();

    // ✨ [수정] sol_space 0 -> 2
    LPROBOT_POSE ik_solution = GlobalDrfl.ikin(target_posx, 2);
    if (ik_solution == nullptr)
    {
        qWarning() << "[ROBOT_THREAD] IK CHECK FAILED. Pose is unreachable:";
        qWarning() << "  - Pos(mm):" << position_mm << "Ori(deg):" << orientation_deg;
        qWarning() << "  - Move command CANCELED.";
        return;
    }

    qDebug() << "[ROBOT_THREAD] IK Check OK. Executing movel command.";
    qDebug() << "  - Solution (J1-J6):"
             << ik_solution->_fPosition[0] << "," << ik_solution->_fPosition[1] << ","
             << ik_solution->_fPosition[2] << "," << ik_solution->_fPosition[3] << ","
             << ik_solution->_fPosition[4] << "," << ik_solution->_fPosition[5];

    float velx[2] = {100.0f, 60.0f};
    float accx[2] = {200.0f, 120.0f};

    qDebug() << "[ROBOT_THREAD] Moving to Pos(mm):" << target_posx[0] << target_posx[1] << target_posx[2]
             << "Rot(deg):" << target_posx[3] << target_posx[4] << target_posx[5];

    if (!GlobalDrfl.movel(target_posx, velx, accx)) {
        qWarning() << "[ROBOT_THREAD] movel command failed AFTER IK check (e.g., collision, E-Stop).";
    }
}
void RobotController::onRobotPickAndReturn(const QVector3D& target_pos_mm, const QVector3D& target_ori_deg,
                                           const QVector3D& approach_pos_mm, const QVector3D& approach_ori_deg)
{
    GlobalDrfl.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
    GlobalDrfl.set_robot_system(ROBOT_SYSTEM_REAL);

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

    // ✨ [수정] sol_space 0 -> 2
    LPROBOT_POSE ik_solution = GlobalDrfl.ikin(final_posx, 2);
    if (ik_solution == nullptr) {
        qWarning() << "[ROBOT_THREAD] IK CHECK FAILED for grasp pose. Sequence Canceled.";
        qWarning() << "  - Pos(mm):" << target_pos_mm << "Ori(deg):" << target_ori_deg;
        return;
    }
    qDebug() << "[ROBOT_THREAD] Grasp pose IK Check OK.";
    qDebug() << "  - Solution (J1-J6):"
             << ik_solution->_fPosition[0] << "," << ik_solution->_fPosition[1] << ","
             << ik_solution->_fPosition[2] << "," << ik_solution->_fPosition[3] << ","
             << ik_solution->_fPosition[4] << "," << ik_solution->_fPosition[5];


    if (GlobalDrfl.movel(final_posx, velx_down, accx_down)) {

        QThread::msleep(100);
        int timeout_ms = 10000;
        int elapsed_ms = 0;
        while (GlobalDrfl.GetRobotState() != STATE_STANDBY) {
            if (elapsed_ms > timeout_ms) {
                qWarning() << "[ROBOT_THREAD] Timeout: Pick move did not complete.";
                GlobalDrfl.MoveStop(STOP_TYPE_SLOW);
                return;
            }
            QThread::msleep(100);
            elapsed_ms += 100;
        }
        qDebug() << "[ROBOT_THREAD] Pick Move Complete.";

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
    GlobalDrfl.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
    GlobalDrfl.set_robot_system(ROBOT_SYSTEM_REAL);

    if (!g_bHasControlAuthority || GlobalDrfl.GetRobotState() != STATE_STANDBY) {
        qWarning() << "[ROBOT_THREAD] Cannot move: No control authority or not in STANDBY.";
        return;
    }
    qInfo() << "[ROBOT_THREAD] Moving to Reset Position.";
    float target_posx[6] = {260,0,350,0,138,0};

    // ✨ [수정] sol_space 0 -> 2
    if (GlobalDrfl.ikin(target_posx, 2) == nullptr) {
        qCritical() << "[ROBOT_THREAD] CRITICAL: IK CHECK FAILED for RESET POSE.";
        return;
    }

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

bool RobotController::moveToPositionAndWait(const QVector3D& pos_mm, const QVector3D& ori_deg)
{
    GlobalDrfl.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
    GlobalDrfl.set_robot_system(ROBOT_SYSTEM_REAL);

    if (!g_bHasControlAuthority || GlobalDrfl.GetRobotState() != STATE_STANDBY) {
        qWarning() << "[ROBOT_THREAD] Cannot move: No control authority or not in STANDBY.";
        return false;
    }

    float target_posx[6];
    target_posx[0] = pos_mm.x(); target_posx[1] = pos_mm.y(); target_posx[2] = pos_mm.z();
    target_posx[3] = ori_deg.x(); target_posx[4] = ori_deg.y(); target_posx[5] = ori_deg.z();

    // 1. IK 체크
    // ✨ [수정] sol_space 0 -> 2
    LPROBOT_POSE solution_pose = GlobalDrfl.ikin(target_posx, 2);

    if (solution_pose == nullptr)
    {
        qWarning() << "[ROBOT_THREAD] IK CHECK FAILED (ikin returned nullptr). Pose is unreachable:";
        qWarning() << "  - Pos(mm):" << pos_mm << "Ori(deg):" << ori_deg;
        qWarning() << "  - Move command CANCELED.";
        return false;
    }

    qDebug() << "[ROBOT_THREAD] IK Check OK. Moving to Pos:" << pos_mm << "Ori:" << ori_deg;
    qDebug() << "  - Solution (J1-J6):"
             << solution_pose->_fPosition[0] << "," << solution_pose->_fPosition[1] << ","
             << solution_pose->_fPosition[2] << "," << solution_pose->_fPosition[3] << ","
             << solution_pose->_fPosition[4] << "," << solution_pose->_fPosition[5];


    float velx[2] = {80.0f, 40.0f};
    float accx[2] = {80.0f, 40.0f};

    // 2. 이동 명령
    if (!GlobalDrfl.movel(target_posx, velx, accx)) {
        qWarning() << "[ROBOT] Move command(movel) failed to send!";
        return false;
    }

    // 3. 이동 완료 대기 (STATE_STANDBY가 될 때까지)
    QThread::msleep(100);

    int timeout_ms = 10000;
    int elapsed_ms = 0;
    while (GlobalDrfl.GetRobotState() != STATE_STANDBY) {
        if (elapsed_ms > timeout_ms) {
            qWarning() << "[ROBOT_THREAD] Timeout: Move did not complete in 10s.";
            GlobalDrfl.MoveStop(STOP_TYPE_SLOW);
            return false;
        }
        QThread::msleep(100);
        elapsed_ms += 100;
    }

    qDebug() << "[ROBOT_THREAD] Move Complete (State is STANDBY).";
    return true;
}

// ✨ [수정] Lift-Rotate-Place 시퀀스: set_robot_system 추가 및 대기 로직 수정
void RobotController::onLiftRotatePlaceSequence(const QVector3D& lift_pos_mm, const QVector3D& lift_ori_deg,
                                                const QVector3D& rotate_pos_mm, const QVector3D& rotate_ori_deg,
                                                const QVector3D& place_pos_mm, const QVector3D& place_ori_deg)
{
    // ✨ [추가] 이 스레드에서 실제 로봇을 제어하도록 명시
    GlobalDrfl.set_robot_system(ROBOT_SYSTEM_REAL);

    if (!g_bHasControlAuthority || GlobalDrfl.GetRobotState() != STATE_STANDBY) {
        qWarning() << "[ROBOT] Cannot start sequence: No control authority or not in STANDBY.";
        return;
    }

    qDebug() << "[ROBOT] ========== Starting Lift-Rotate-Place Sequence (Manual) ==========";

    // Step 1: 3cm 위로 올라가기
    qDebug() << "[ROBOT] Step 1/5: Lifting 3cm up";
    if (!moveToPositionAndWait(lift_pos_mm, lift_ori_deg)) {
        qWarning() << "[ROBOT] Step 1 (Lift) FAILED. Aborting sequence."; return;
    }
    // msleep(2000) 삭제

    // Step 2: 회전하기 (같은 높이에서)
    qDebug() << "[ROBOT] Step 2/5: Rotating to Y-aligned pose (Rz =" << rotate_ori_deg.z() << "deg)";
    if (!moveToPositionAndWait(rotate_pos_mm, rotate_ori_deg)) {
        qWarning() << "[ROBOT] Step 2 (Rotate) FAILED. Aborting sequence."; return;
    }
    // msleep(2000) 삭제

    // Step 3: 원래 높이로 내려가기
    qDebug() << "[ROBOT] Step 3/5: Placing back down to original height";
    if (!moveToPositionAndWait(place_pos_mm, place_ori_deg)) {
        qWarning() << "[ROBOT] Step 3 (Place) FAILED. Aborting sequence."; return;
    }
    // msleep(2000) 삭제

    // Step 4: 그리퍼 열기
    qDebug() << "[ROBOT] Step 4/5: Opening gripper (Release)";
    onGripperAction(0);
    QThread::msleep(2000); // 그리퍼 동작은 msleep 유지

    // Step 5: 다시 위로 올라가기
    qDebug() << "[ROBOT] Step 5/5: Lifting back up after release";
    if (!moveToPositionAndWait(rotate_pos_mm, place_ori_deg)) {
        qWarning() << "[ROBOT] Step 5 (Lift after release) FAILED.";
    }
    // msleep(1000) 삭제

    qDebug() << "[ROBOT] ========== Lift-Rotate-Place Sequence Completed! ==========";
}


// ✨ [수정] 전체 자동 시퀀스: set_robot_system 추가 및 대기 로직 수정
void RobotController::onFullPickAndPlaceSequence(
    const QVector3D& pre_grasp_pos_mm, const QVector3D& pre_grasp_ori_deg,
    const QVector3D& grasp_pos_mm, const QVector3D& grasp_ori_deg,
    const QVector3D& lift_pos_mm, const QVector3D& lift_ori_deg,
    const QVector3D& rotate_pos_mm, const QVector3D& rotate_ori_deg,
    const QVector3D& place_pos_mm, const QVector3D& place_ori_deg)
{
    // ✨ [추가] 이 스레드에서 실제 로봇을 제어하도록 명시
    GlobalDrfl.set_robot_system(ROBOT_SYSTEM_REAL);

    if (!g_bHasControlAuthority || GlobalDrfl.GetRobotState() != STATE_STANDBY) {
        qWarning() << "[ROBOT_SEQ] Cannot start full sequence: No control authority or not in STANDBY.";
        return;
    }

    qDebug() << "[ROBOT_SEQ] ========== Starting Full Automated Sequence ==========";

    // Step 1: (M) 그리퍼 열기
    qDebug() << "[ROBOT_SEQ] Step 1/9: Opening Gripper (for M)";
    onGripperAction(0);
    QThread::msleep(1500); // 그리퍼 동작

    // Step 2: (M) Pre-Grasp 위치로 이동
    qDebug() << "[ROBOT_SEQ] Step 2/9: Moving to Pre-Grasp Pose (M)";
    if (!moveToPositionAndWait(pre_grasp_pos_mm, pre_grasp_ori_deg)) {
        qWarning() << "[ROBOT_SEQ] Step 2 (Pre-Grasp) FAILED. Aborting sequence."; return;
    }
    // msleep(2000) 삭제

    // Step 3: (D) Grasp 위치로 하강
    qDebug() << "[ROBOT_SEQ] Step 3/9: Moving to Grasp Pose (D-part 1)";
    if (!moveToPositionAndWait(grasp_pos_mm, grasp_ori_deg)) {
        qWarning() << "[ROBOT_SEQ] Step 3 (Grasp) FAILED. Aborting sequence."; return;
    }
    // msleep(2000) 삭제

    // Step 4: (D) 그리퍼 닫기
    qDebug() << "[ROBOT_SEQ] Step 4/9: Closing Gripper (D-part 2)";
    onGripperAction(1);
    QThread::msleep(1500); // 그리퍼 동작

    // Step 5: (Move) 3cm 위로 올라가기
    qDebug() << "[ROBOT_SEQ] Step 5/9: Lifting object (Move-part 1)";
    if (!moveToPositionAndWait(lift_pos_mm, lift_ori_deg)) {
        qWarning() << "[ROBOT_SEQ] Step 5 (Lift) FAILED. Aborting sequence."; return;
    }
    // msleep(2000) 삭제

    // Step 6: (Move) 회전하기 (같은 높이에서)
    qDebug() << "[ROBOT_SEQ] Step 6/9: Rotating object (Move-part 2)";
    if (!moveToPositionAndWait(rotate_pos_mm, rotate_ori_deg)) {
        qWarning() << "[ROBOT_SEQ] Step 6 (Rotate) FAILED. Aborting sequence."; return;
    }
    // msleep(2000) 삭제

    // Step 7: (Move) 원래 높이로 내려가기
    qDebug() << "[ROBOT_SEQ] Step 7/9: Placing object (Move-part 3)";
    if (!moveToPositionAndWait(place_pos_mm, place_ori_deg)) {
        qWarning() << "[ROBOT_SEQ] Step 7 (Place) FAILED. Aborting sequence."; return;
    }
    // msleep(2000) 삭제

    // Step 8: (Move) 그리퍼 열기
    qDebug() << "[ROBOT_SEQ] Step 8/9: Opening gripper (Move-part 4)";
    onGripperAction(0);
    QThread::msleep(1500); // 그리퍼 동작

    // Step 9: (Move) 다시 위로 올라가기
    qDebug() << "[ROBOT_SEQ] Step 9/9: Lifting back up (Move-part 5)";
    if (!moveToPositionAndWait(rotate_pos_mm, place_ori_deg)) {
        qWarning() << "[ROBOT_SEQ] Step 9 (Lift after place) FAILED.";
    }
    // msleep(1000) 삭제

    qDebug() << "[ROBOT_SEQ] ========== Full Automated Sequence Completed! ==========";
}


// ✨ [수정] onApproachThenGrasp: set_robot_system 추가 및 대기 로직 수정
void RobotController::onApproachThenGrasp(const QVector3D& approach_pos_mm, const QVector3D& final_pos_mm, const QVector3D& orientation_deg)
{
    GlobalDrfl.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
    GlobalDrfl.set_robot_system(ROBOT_SYSTEM_REAL);

    if (!g_bHasControlAuthority || GlobalDrfl.GetRobotState() != STATE_STANDBY) {
        qWarning() << "[ROBOT_SEQ] Cannot start Approach-Then-Grasp: No control or not STANDBY.";
        return;
    }

    qInfo() << "[ROBOT_SEQ] ========== Starting Approach-Then-Grasp & Lift (Grasp Handle) ==========";

    // Step 1: 5cm 뒤(접근 위치)로 이동
    qDebug() << "[ROBOT] Step 1/4: Moving to Approach Pose (-5cm)";
    if (!moveToPositionAndWait(approach_pos_mm, orientation_deg)) {
        qWarning() << "[ROBOT_SEQ] Step 1 (Approach) FAILED. Aborting sequence.";
        return;
    }

    // Step 2: 최종 목표 위치로 이동 (느리게)
    qDebug() << "[ROBOT] Step 2/4: Moving to Final Grasp Pose (Slowly)";
    float velx_slow[2] = {50.0f, 30.0f}; // V: 50, W: 30
    float accx_slow[2] = {50.0f, 30.0f}; // A: 50, W: 30

    float target_posx[6];
    target_posx[0] = final_pos_mm.x(); target_posx[1] = final_pos_mm.y(); target_posx[2] = final_pos_mm.z();
    target_posx[3] = orientation_deg.x(); target_posx[4] = orientation_deg.y(); target_posx[5] = orientation_deg.z();

    // ✨ [수정] sol_space 0 -> 2
    LPROBOT_POSE solution_pose = GlobalDrfl.ikin(target_posx, 2);
    if (solution_pose == nullptr) {
        qWarning() << "[ROBOT_SEQ] Step 2 (Final Grasp Pose) IK CHECK FAILED. Aborting sequence.";
        qWarning() << "  - Pos(mm):" << final_pos_mm << "Ori(deg):" << orientation_deg;
        return;
    }
    qDebug() << "[ROBOT_THREAD] Step 2 IK Check OK.";
    qDebug() << "  - Solution (J1-J6):"
             << solution_pose->_fPosition[0] << "," << solution_pose->_fPosition[1] << ","
             << solution_pose->_fPosition[2] << "," << solution_pose->_fPosition[3] << ","
             << solution_pose->_fPosition[4] << "," << solution_pose->_fPosition[5];


    if (GlobalDrfl.GetRobotState() != STATE_STANDBY) {
        qWarning() << "[ROBOT_SEQ] Step 2: Robot not in STANDBY. Aborting.";
        return;
    }

    if (!GlobalDrfl.movel(target_posx, velx_slow, accx_slow)) {
        qWarning() << "[ROBOT] Final move command(movel) failed to send!";
        qInfo() << "[ROBOT_SEQ] ========== Sequence Aborted ========== ";
        return;
    }

    // Step 2 이동 완료 대기
    QThread::msleep(100);
    int timeout_ms = 10000;
    int elapsed_ms = 0;
    while (GlobalDrfl.GetRobotState() != STATE_STANDBY) {
        if (elapsed_ms > timeout_ms) {
            qWarning() << "[ROBOT_THREAD] Timeout: Step 2 Move did not complete.";
            GlobalDrfl.MoveStop(STOP_TYPE_SLOW);
            return;
        }
        QThread::msleep(100);
        elapsed_ms += 100;
    }
    qDebug() << "[ROBOT_THREAD] Step 2 Move Complete.";


    // Step 3: 그리퍼 닫기
    qDebug() << "[ROBOT] Step 3/4: Closing Gripper";
    onGripperAction(1); // 1 = Close
    QThread::msleep(1500); // 3. 그리퍼 닫힘 대기

    // Step 4: 글로벌 Z축 기준 10cm (100mm) 위로 이동
    qDebug() << "[ROBOT] Step 4/4: Lifting up 10cm (Global Z)";
    QVector3D lift_pos_mm = final_pos_mm + QVector3D(0.0f, 0.0f, 100.0f);

    if (!moveToPositionAndWait(lift_pos_mm, orientation_deg)) {
        qWarning() << "[ROBOT_SEQ] Step 4 (Lift) FAILED.";
    }

    qInfo() << "[ROBOT_SEQ] ========== Approach-Then-Grasp & Lift Completed! ==========";
}
