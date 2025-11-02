#include "robotsequencer.h"
#include <QDebug>
#include <QThread>
#include "DRFLEx.h" // extern 전역 변수 사용을 위해 포함

// 전역 변수 접근 (RobotController와 동일한 스레드)
extern CDRFLEx GlobalDrfl;
extern bool g_bHasControlAuthority;

RobotSequencer::RobotSequencer(QObject *parent)
    : QObject(parent)
    , m_robotController(nullptr)
{
}

void RobotSequencer::setRobotController(RobotController *controller)
{
    m_robotController = controller;
}


//
// --- 이하 코드는 robotcontroller.cpp에서 그대로 옮겨온 시퀀스 함수들 ---
//
// (호출 방식만 m_robotController->... 로 변경)
//

void RobotSequencer::onRobotPickAndReturn(const QVector3D& target_pos_mm, const QVector3D& target_ori_deg,
                                          const QVector3D& approach_pos_mm, const QVector3D& approach_ori_deg)
{
    if (!m_robotController) {
        qWarning() << "[SEQ] RobotController not set!"; return;
    }

    GlobalDrfl.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
    GlobalDrfl.set_robot_system(ROBOT_SYSTEM_REAL);

    if (!g_bHasControlAuthority || GlobalDrfl.GetRobotState() != STATE_STANDBY) {
        qWarning() << "[ROBOT_THREAD] Cannot start sequence: No control authority or not in STANDBY.";
        return;
    }
    qInfo() << "[ROBOT_THREAD] ========== D Key: Pick Only (NO Return) ==========";

    // Step 1: 목표 위치로 하강 (moveToPositionAndWait 사용)
    qDebug() << "[ROBOT] Step 1/2: Descending to grasp position:" << target_pos_mm;
    if (m_robotController->moveToPositionAndWait(target_pos_mm, target_ori_deg)) {

        // Step 2: 그리퍼 닫기
        qDebug() << "[ROBOT] Step 2/2: Closing gripper (Grasp)";
        m_robotController->onGripperAction(1);
        QThread::msleep(1500);

        qInfo() << "[ROBOT_THREAD] ========== Pick completed! Object grasped. Ready for MoveButton. ==========";
    } else {
        qWarning() << "[ROBOT_THREAD] ERROR: Move down to target failed.";
    }
}


void RobotSequencer::onLiftRotatePlaceSequence(const QVector3D& lift_pos_mm, const QVector3D& lift_ori_deg,
                                               const QVector3D& rotate_pos_mm, const QVector3D& rotate_ori_deg,
                                               const QVector3D& place_pos_mm, const QVector3D& place_ori_deg)
{
    if (!m_robotController) {
        qWarning() << "[SEQ] RobotController not set!"; return;
    }

    GlobalDrfl.set_robot_system(ROBOT_SYSTEM_REAL);

    if (!g_bHasControlAuthority || GlobalDrfl.GetRobotState() != STATE_STANDBY) {
        qWarning() << "[ROBOT] Cannot start sequence: No control authority or not in STANDBY.";
        return;
    }

    qDebug() << "[ROBOT] ========== Starting Lift-Rotate-Place Sequence (Manual) ==========";

    // Step 1: 3cm 위로 올라가기
    qDebug() << "[ROBOT] Step 1/5: Lifting 3cm up";
    if (!m_robotController->moveToPositionAndWait(lift_pos_mm, lift_ori_deg)) {
        qWarning() << "[ROBOT] Step 1 (Lift) FAILED. Aborting sequence."; return;
    }

    // Step 2: 회전하기 (같은 높이에서)
    qDebug() << "[ROBOT] Step 2/5: Rotating to Y-aligned pose (Rz =" << rotate_ori_deg.z() << "deg)";
    if (!m_robotController->moveToPositionAndWait(rotate_pos_mm, rotate_ori_deg)) {
        qWarning() << "[ROBOT] Step 2 (Rotate) FAILED. Aborting sequence."; return;
    }

    // Step 3: 원래 높이로 내려가기
    qDebug() << "[ROBOT] Step 3/5: Placing back down to original height";
    if (!m_robotController->moveToPositionAndWait(place_pos_mm, place_ori_deg)) {
        qWarning() << "[ROBOT] Step 3 (Place) FAILED. Aborting sequence."; return;
    }

    // Step 4: 그리퍼 열기
    qDebug() << "[ROBOT] Step 4/5: Opening gripper (Release)";
    m_robotController->onGripperAction(0);
    QThread::msleep(2000); // 그리퍼 동작은 msleep 유지

    // Step 5: 다시 위로 올라가기
    qDebug() << "[ROBOT] Step 5/5: Lifting back up after release";
    if (!m_robotController->moveToPositionAndWait(rotate_pos_mm, place_ori_deg)) {
        qWarning() << "[ROBOT] Step 5 (Lift after release) FAILED.";
    }

    qDebug() << "[ROBOT] ========== Lift-Rotate-Place Sequence Completed! ==========";
}


void RobotSequencer::onFullPickAndPlaceSequence(
    const QVector3D& pre_grasp_pos_mm, const QVector3D& pre_grasp_ori_deg,
    const QVector3D& grasp_pos_mm, const QVector3D& grasp_ori_deg,
    const QVector3D& lift_pos_mm, const QVector3D& lift_ori_deg,
    const QVector3D& rotate_pos_mm, const QVector3D& rotate_ori_deg,
    const QVector3D& place_pos_mm, const QVector3D& place_ori_deg)
{
    if (!m_robotController) {
        qWarning() << "[SEQ] RobotController not set!"; return;
    }

    GlobalDrfl.set_robot_system(ROBOT_SYSTEM_REAL);

    if (!g_bHasControlAuthority || GlobalDrfl.GetRobotState() != STATE_STANDBY) {
        qWarning() << "[ROBOT_SEQ] Cannot start full sequence: No control authority or not in STANDBY.";
        return;
    }

    qDebug() << "[ROBOT_SEQ] ========== Starting Full Automated Sequence ==========";

    // Step 1: (M) 그리퍼 열기
    qDebug() << "[ROBOT_SEQ] Step 1/9: Opening Gripper (for M)";
    m_robotController->onGripperAction(0);
    QThread::msleep(1500); // 그리퍼 동작

    // Step 2: (M) Pre-Grasp 위치로 이동
    qDebug() << "[ROBOT_SEQ] Step 2/9: Moving to Pre-Grasp Pose (M)";
    if (!m_robotController->moveToPositionAndWait(pre_grasp_pos_mm, pre_grasp_ori_deg)) {
        qWarning() << "[ROBOT_SEQ] Step 2 (Pre-Grasp) FAILED. Aborting sequence."; return;
    }

    // Step 3: (D) Grasp 위치로 하강
    qDebug() << "[ROBOT_SEQ] Step 3/9: Moving to Grasp Pose (D-part 1)";
    if (!m_robotController->moveToPositionAndWait(grasp_pos_mm, grasp_ori_deg)) {
        qWarning() << "[ROBOT_SEQ] Step 3 (Grasp) FAILED. Aborting sequence."; return;
    }

    // Step 4: (D) 그리퍼 닫기
    qDebug() << "[ROBOT_SEQ] Step 4/9: Closing Gripper (D-part 2)";
    m_robotController->onGripperAction(1);
    QThread::msleep(1500); // 그리퍼 동작

    // Step 5: (Move) 3cm 위로 올라가기
    qDebug() << "[ROBOT_SEQ] Step 5/9: Lifting object (Move-part 1)";
    if (!m_robotController->moveToPositionAndWait(lift_pos_mm, lift_ori_deg)) {
        qWarning() << "[ROBOT_SEQ] Step 5 (Lift) FAILED. Aborting sequence."; return;
    }

    // Step 6: (Move) 회전하기 (같은 높이에서)
    qDebug() << "[ROBOT_SEQ] Step 6/9: Rotating object (Move-part 2)";
    if (!m_robotController->moveToPositionAndWait(rotate_pos_mm, rotate_ori_deg)) {
        qWarning() << "[ROBOT_SEQ] Step 6 (Rotate) FAILED. Aborting sequence."; return;
    }

    // Step 7: (Move) 원래 높이로 내려가기
    qDebug() << "[ROBOT_SEQ] Step 7/9: Placing object (Move-part 3)";
    if (!m_robotController->moveToPositionAndWait(place_pos_mm, place_ori_deg)) {
        qWarning() << "[ROBOT_SEQ] Step 7 (Place) FAILED. Aborting sequence."; return;
    }

    // Step 8: (Move) 그리퍼 열기
    qDebug() << "[ROBOT_SEQ] Step 8/9: Opening gripper (Move-part 4)";
    m_robotController->onGripperAction(0);
    QThread::msleep(1500); // 그리퍼 동작

    // Step 9: (Move) 다시 위로 올라가기
    qDebug() << "[ROBOT_SEQ] Step 9/9: Lifting back up (Move-part 5)";
    if (!m_robotController->moveToPositionAndWait(rotate_pos_mm, place_ori_deg)) {
        qWarning() << "[ROBOT_SEQ] Step 9 (Lift after place) FAILED.";
    }

    qDebug() << "[ROBOT_SEQ] ========== Full Automated Sequence Completed! ==========";
}


void RobotSequencer::onApproachThenGrasp(const QVector3D& approach_pos_mm, const QVector3D& final_pos_mm, const QVector3D& orientation_deg)
{
    if (!m_robotController) {
        qWarning() << "[SEQ] RobotController not set!"; return;
    }

    GlobalDrfl.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
    GlobalDrfl.set_robot_system(ROBOT_SYSTEM_REAL);

    if (!g_bHasControlAuthority || GlobalDrfl.GetRobotState() != STATE_STANDBY) {
        qWarning() << "[ROBOT_SEQ] Cannot start Approach-Then-Grasp: No control or not STANDBY.";
        return;
    }

    qInfo() << "[ROBOT_SEQ] ========== Starting Approach-Then-Grasp & Lift (Grasp Handle) ==========";
    m_robotController->onGripperAction(0);
    // Step 1: 5cm 뒤(접근 위치)로 이동
    qDebug() << "[ROBOT] Step 1/4: Moving to Approach Pose (-5cm)";
    if (!m_robotController->moveToPositionAndWait(approach_pos_mm, orientation_deg)) {
        qWarning() << "[ROBOT_SEQ] Step 1 (Approach) FAILED. Aborting sequence.";
        return;
    }

    // Step 2: 최종 목표 위치로 이동 (느리게)
    // moveToPositionAndWait는 기본 속도를 사용하므로,
    // 이 부분은 onMoveRobot을 호출하고 직접 기다리는 방식을 사용합니다.
    qDebug() << "[ROBOT] Step 2/4: Moving to Final Grasp Pose (Slowly)";
    float velx_slow[2] = {50.0f, 30.0f}; // V: 50, W: 30
    float accx_slow[2] = {50.0f, 30.0f}; // A: 50, W: 30

    float target_posx[6];
    target_posx[0] = final_pos_mm.x(); target_posx[1] = final_pos_mm.y(); target_posx[2] = final_pos_mm.z();
    target_posx[3] = orientation_deg.x(); target_posx[4] = orientation_deg.y(); target_posx[5] = orientation_deg.z();

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
    m_robotController->onGripperAction(1); // 1 = Close
    QThread::msleep(1500); // 3. 그리퍼 닫힘 대기

    // Step 4: 글로벌 Z축 기준 10cm (100mm) 위로 이동
    qDebug() << "[ROBOT] Step 4/4: Lifting up 10cm (Global Z)";
    QVector3D lift_pos_mm = final_pos_mm + QVector3D(0.0f, 0.0f, 100.0f);

    if (!m_robotController->moveToPositionAndWait(lift_pos_mm, orientation_deg)) {
        qWarning() << "[ROBOT_SEQ] Step 4 (Lift) FAILED.";
    }

    qInfo() << "[ROBOT_SEQ] ========== Approach-Then-Grasp & Lift Completed! ==========";
}
