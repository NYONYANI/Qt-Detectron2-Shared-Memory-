#include "robotsequencer.h"
#include <QDebug>
#include <QThread>
#include "DRFLEx.h" // extern 전역 변수 사용을 위해 포함
#include <QApplication>
#include <QMatrix4x4> // "invalid use of incomplete type" 오류 해결

// 전역 변수 접근 (RobotController와 동일한 스레드)
extern CDRFLEx GlobalDrfl;
extern bool g_bHasControlAuthority;

RobotSequencer::RobotSequencer(QObject *parent)
    : QObject(parent)
    , m_robotController(nullptr)
    , m_autoState(Idle) // ✨ [추가]
    , m_visionWaitLoop(nullptr) // ✨ [추가]
    , m_moveWaitLoop(nullptr)
{
}

void RobotSequencer::setRobotController(RobotController *controller)
{
    m_robotController = controller;
}


//
// ✨ [수정] 자동화 시퀀스 메인 함수 (두 번째 필터 토글 제거)
//
void RobotSequencer::onStartFullAutomation()
{
    if (m_autoState != Idle) {
        qWarning() << "[SEQ] 자동화 시퀀스가 이미 실행 중입니다.";
        return;
    }
    if (!m_robotController) {
        qWarning() << "[SEQ] RobotController가 설정되지 않았습니다!";
        return;
    }

    qInfo() << "[SEQ] ========== 전체 자동화 시퀀스 시작 ==========";

    // --- 1. Capture ---
    m_autoState = Step1_Capture;
    qInfo() << "[SEQ] 1/9: Capture 요청 (비전 작업 대기...)";
    m_visionWaitLoop = new QEventLoop();
    emit requestVisionCapture(); // RealSenseWidget::captureAndProcess(true) 슬롯 호출
    m_visionWaitLoop->exec();    // onVisionTaskComplete()가 호출될 때까지 대기
    delete m_visionWaitLoop; m_visionWaitLoop = nullptr;
    if (m_autoState == Idle) { qWarning() << "[SEQ] 시퀀스 중단 (Capture 1)"; return; }
    qInfo() << "[SEQ] 1/9: Capture 완료.";
    qInfo() << "[SEQ] 1/9: 캡처 적용을 위해 이벤트 루프 처리 강제...";
    QApplication::processEvents();
    QThread::msleep(500); // 안정화 대기

    // --- 2. 필터 적용 (키 1, 2, 3) ---
    m_autoState = Step2_Filter;
    qInfo() << "[SEQ] 2/9: 필터 적용 (Toggle Mask, Denoise, Z-Filter)";
    emit requestToggleMask();
    emit requestToggleDenoise();
    emit requestToggleZFilter();
    QThread::msleep(100); // UI가 필터를 적용할 시간을 줌

    // --- 3. Move Viewpoint (계산) ---
    m_autoState = Step3_CalcView;
    qInfo() << "[SEQ] 3/9: Move Viewpoint 계산 요청 (비전 작업 대기...)";
    m_visionWaitLoop = new QEventLoop();
    emit requestVisionMoveViewpoint(); // RealSenseWidget::onCalculateHandleViewPose() 슬롯 호출
    m_visionWaitLoop->exec();
    delete m_visionWaitLoop; m_visionWaitLoop = nullptr;
    if (m_autoState == Idle) { qWarning() << "[SEQ] 시퀀스 중단 (Calc View)"; return; }
    qInfo() << "[SEQ] 3/9: Viewpoint 계산 완료.";
    QThread::msleep(500);

    // --- 4. Move Viewpoint (이동) ---
    m_autoState = Step4_MoveView;
    qInfo() << "[SEQ] 4/9: Move Viewpoint 이동 요청 (로봇 이동 대기...)";

    // ✨ [수정]
    // emit requestMoveToViewpoint()는 즉시 리턴하고 이동을 큐에 넣습니다.
    // 5단계의 캡처가 실행되기 *전에* 이동이 완료되어야 하므로,
    // m_moveWaitLoop를 사용하여 RobotController의 'moveFinished' 시그널을 기다립니다.

    m_moveWaitLoop = new QEventLoop();
    emit requestMoveToViewpoint(); // 1. 메인 스레드에 이동 계산 및 로봇 스레드로의 이동 명령 큐잉을 요청

    // 2. 로봇 스레드(현재 스레드)의 이벤트 루프를 실행함.
    //    이 루프는 메인 스레드로부터 큐에 쌓인 'moveToPositionAndWait'을 실행함.
    //    'moveToPositionAndWait'이 완료되면 'moveFinished' 시그널을 보내고,
    //    'onMoveTaskComplete' 슬롯이 'm_moveWaitLoop->quit()'을 호출함.
    m_moveWaitLoop->exec();

    delete m_moveWaitLoop; m_moveWaitLoop = nullptr;
    if (m_autoState == Idle) { qWarning() << "[SEQ] 시퀀스 중단 (Move View)"; return; }
    qInfo() << "[SEQ] 4/9: Viewpoint 이동 완료.";
    // msleep(1000)은 이제 moveToPositionAndWait 내부에 있으므로 제거 가능 (유지해도 무방)
    QThread::msleep(500); // 이동 후 안정화 대기


    // --- 5. Capture (두 번째) ---
    m_autoState = Step5_Capture;
    qInfo() << "[SEQ] 5/9: 두 번째 Capture 요청 (비전 작업 대기...)";
    m_visionWaitLoop = new QEventLoop();
    emit requestVisionCapture();
    m_visionWaitLoop->exec();
    delete m_visionWaitLoop; m_visionWaitLoop = nullptr;
    if (m_autoState == Idle) { qWarning() << "[SEQ] 시퀀스 중단 (Capture 2)"; return; }
    qInfo() << "[SEQ] 5/9: 두 번째 Capture 완료.";

    qInfo() << "[SEQ] 5/9: 캡처 적용을 위해 이벤트 루프 처리 강제...";
    QApplication::processEvents();
    QThread::msleep(500);

    // --- 7. View Handle (계산) ---
    m_autoState = Step6_CalcHandle;
    qInfo() << "[SEQ] 6/9: View Handle 계산 요청 (비전 작업 대기...)"; // <-- 단계 번호 수정
    m_visionWaitLoop = new QEventLoop();
    emit requestVisionHandlePlot(); // RealSenseWidget::onShowHandlePlot() 슬롯 호출
    m_visionWaitLoop->exec();
    delete m_visionWaitLoop; m_visionWaitLoop = nullptr;
    if (m_autoState == Idle) { qWarning() << "[SEQ] 시퀀스 중단 (Calc Handle)"; return; }
    qInfo() << "[SEQ] 6/9: View Handle 계산 완료."; // <-- 단계 번호 수정
    QThread::msleep(500);

    // --- 8. Reset Position (이동) ---
    m_autoState = Step7_Reset;
    qInfo() << "[SEQ] 7/9: Reset Position 이동 요청 (로봇 이동 대기...)"; // <-- 단계 번호 수정
    m_robotController->onResetPosition(); // 이 함수는 블로킹
    qInfo() << "[SEQ] 7/9: Reset Position 이동 완료."; // <-- 단계 번호 수정
    QThread::msleep(1000);

    // --- 9. Grasp Handle (이동) ---
    m_autoState = Step8_Grasp;
    qInfo() << "[SEQ] 8/9: Grasp Handle 이동 요청 (로봇 이동 대기...)"; // <-- 단계 번호 수정
    emit requestGraspHandle();
    qInfo() << "[SEQ] 8/9: Grasp Handle 이동 완료."; // <-- 단계 번호 수정
    QThread::msleep(1000);

    // --- 10. (추가) 최종 리셋 ---
    m_autoState = Step9_FinalReset;
    qInfo() << "[SEQ] 9/9: 최종 Reset Position 이동 요청 (로봇 이동 대기...)"; // <-- 단계 번호 수정
    m_robotController->onResetPosition();
    qInfo() << "[SEQ] 9/9: 최종 Reset Position 이동 완료."; // <-- 단계 번호 수정


    qInfo() << "[SEQ] ========== 전체 자동화 시퀀스 완료 ==========";
    m_autoState = Idle;
    emit automationFinished(); // MainWindow UI 활성화를 위해 시그널 전송
}

void RobotSequencer::onVisionTaskComplete()
{
    // ✨ [수정] 시퀀스가 중단되었을 때(예: 사용자가 다른 버튼을 누름) 루프가 없으면 quit()을 호출하지 않도록 함
    qDebug() << "[SEQ] 비전 작업 완료 신호 수신 (State:" << m_autoState << ")";
    if (m_visionWaitLoop && m_visionWaitLoop->isRunning()) {
        m_visionWaitLoop->quit();
    } else if (m_autoState == Idle) {
        qWarning() << "[SEQ] Vision complete signal received, but auto-sequence is already Idle.";
    } else {
        qWarning() << "[SEQ] Vision complete signal received, but no wait loop is running!";
    }
}
void RobotSequencer::onMoveTaskComplete()
{
    qDebug() << "[SEQ] 로봇 이동 완료 신호 수신 (State:" << m_autoState << ")";
    if (m_moveWaitLoop && m_moveWaitLoop->isRunning()) {
        m_moveWaitLoop->quit();
    } else if (m_autoState == Idle) {
        qWarning() << "[SEQ] Move complete signal received, but auto-sequence is already Idle.";
    } else {
        qWarning() << "[SEQ] Move complete signal received, but no wait loop is running!";
    }
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


void RobotSequencer::onApproachThenGrasp(const QVector3D& approach_pos_mm, const QVector3D& final_pos_mm, const QVector3D& orientation_deg,
                                         const QMatrix4x4& hang_pose_matrix) // ✨
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


    // --- ✨ [수정] 걸기(Hang) 시퀀스 자동 실행 ---
    if (hang_pose_matrix.isIdentity()) {
        qWarning() << "[ROBOT_SEQ] No hang pose was provided (matrix is identity).";
        qInfo() << "[ROBOT_SEQ] ========== Sequence Finished (Grasp & Lift only) ==========";
        return; // 기존 동작: 여기서 종료
    }

    // ✨ [사용자 요청] 걸기 동작 직전에 Reset Position으로 이동
    qInfo() << "[ROBOT_SEQ] Moving to Reset Position before starting Hang sequence...";
    m_robotController->onResetPosition(); // 이 함수는 블로킹(동기) 방식입니다.
    qInfo() << "[ROBOT_SEQ] Reset Position complete.";
    QThread::msleep(500); // 0.5초 대기


    qInfo() << "[ROBOT_SEQ] ========== Proceeding to Aligned Hang Sequence... ==========";

    // 1. 걸기 자세(Hang Pose)에서 (A,B,C) 방향 및 (X,Y,Z) 위치 추출
    QVector3D place_pos_m = hang_pose_matrix.column(3).toVector3D();
    QMatrix3x3 rotMat = hang_pose_matrix.toGenericMatrix<3,3>();

    // (A,B,C) 계산 (RobotController의 헬퍼 함수 사용)
    QVector3D oriZYZ = m_robotController->rotationMatrixToEulerAngles(rotMat, "ZYZ");
    float cmdA = oriZYZ.x() + 180.0f;
    float cmdB = -oriZYZ.y();
    float cmdC = oriZYZ.z() + 180.0f;
    while(cmdA > 180.0f) cmdA -= 360.0f; while(cmdA <= -180.0f) cmdA += 360.0f;
    while(cmdB > 180.0f) cmdB -= 360.0f; while(cmdB <= -180.0f) cmdB += 360.0f;
    while(cmdC > 180.0f) cmdC -= 360.0f; while(cmdC <= -180.0f) cmdC += 360.0f;
    QVector3D place_ori_deg(cmdA, cmdB, cmdC);
    QVector3D place_pos_mm = place_pos_m * 1000.0f; // <-- 최종 목표 EF 위치

    // 2. ✨ [사용자 요청 수정] Y축 +10cm (100mm) 중간 지점 계산
    // 월드 좌표계 기준 Y축으로 100mm 오프셋
    QVector3D y_offset_mm(0.0f, 100.0f, 0.0f); // 300mm -> 100mm (10cm)
    QVector3D intermediate_Y_offset_pos_mm = place_pos_mm + y_offset_mm;

    qInfo() << "[ROBOT_SEQ] Original Hang Pos (mm):" << place_pos_mm;
    qInfo() << "[ROBOT_SEQ] Moving to Intermediate Y-Offset Pos (mm):" << intermediate_Y_offset_pos_mm;


    // 3. (Hang Step 1: 수정된 중간 지점(Y+10cm)으로 이동)
    qDebug() << "[ROBOT] Hang Step 1/2: Moving to Intermediate Hang Pose (Y+10cm)";
    if (!m_robotController->moveToPositionAndWait(intermediate_Y_offset_pos_mm, place_ori_deg)) {
        qWarning() << "[ROBOT_SEQ] Hang Step 1 (Intermediate Move) FAILED. Aborting sequence.";
        return;
    }

    // 4. ✨ [사용자 요청 추가] (Hang Step 2: 최종 목표 지점으로 이동)
    qDebug() << "[ROBOT] Hang Step 2/2: Moving to Final Hang Pose";
    if (!m_robotController->moveToPositionAndWait(place_pos_mm, place_ori_deg)) {
        qWarning() << "[ROBOT_SEQ] Hang Step 2 (Final Move) FAILED. Aborting sequence.";
        return;
    }

    // ✨ [사용자 요청] 최종 지점으로 이동 후 시퀀스 종료 (이하 동작 주석 처리)

    // (Hang Step 3: 그리퍼 열기 - 컵 놓기)
    // qDebug() << "[ROBOT] Hang Step 3/3: Opening Gripper (Release)";
    // m_robotController->onGripperAction(0); // 0 = Open
    // QThread::msleep(1500);

    // (Hang Step 4: 후퇴 위치로 이동)
    // (후퇴 위치 계산)
    // QVector3D ef_z_axis = hang_pose_matrix.column(2).toVector3D().normalized();
    // float approach_dist_m = 0.05f; // 5cm
    // QVector3D approach_pos_m = place_pos_m - (ef_z_axis * approach_dist_m);
    // QVector3D retreat_pos_mm = approach_pos_m * 1000.0f; // mm 단위로 변환
    // qDebug() << "[ROBOT] Hang Step 4/4: Moving to Retreat Pose";
    // if (!m_robotController->moveToPositionAndWait(retreat_pos_mm, place_ori_deg)) {
    //     qWarning() << "[ROBOT_SEQ] Hang Step 4 (Retreat) FAILED.";
    // }

    qInfo() << "[ROBOT_SEQ] ========== Full Grasp-to-Hang Sequence Completed! (Stopped at Final Hang Pos) ==========";
}
void RobotSequencer::onAlignHangSequence(const QVector3D& approach_pos_mm,
                                         const QVector3D& place_pos_mm,
                                         const QVector3D& retreat_pos_mm,
                                         const QVector3D& orientation_deg)
{
    if (!m_robotController) {
        qWarning() << "[SEQ] RobotController not set!"; return;
    }

    GlobalDrfl.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
    GlobalDrfl.set_robot_system(ROBOT_SYSTEM_REAL);

    if (!g_bHasControlAuthority || GlobalDrfl.GetRobotState() != STATE_STANDBY) {
        qWarning() << "[ROBOT_SEQ] Cannot start Align-Hang: No control or not STANDBY.";
        return;
    }

    qInfo() << "[ROBOT_SEQ] ========== Starting Align-Hang Sequence (Pole Start) ==========";

    // Step 1: 접근 위치로 이동
    qDebug() << "[ROBOT] Step 1/4: Moving to Approach Pose";
    if (!m_robotController->moveToPositionAndWait(approach_pos_mm, orientation_deg)) {
        qWarning() << "[ROBOT_SEQ] Step 1 (Approach) FAILED. Aborting sequence.";
        return;
    }

    // Step 2: 최종 배치(Place) 위치로 이동 (천천히)
    // (속도를 느리게 하려면 onApproachThenGrasp처럼 별도 movel 호출 필요)
    // (여기서는 표준 속도 사용)
    qDebug() << "[ROBOT] Step 2/4: Moving to Final Place Pose";
    if (!m_robotController->moveToPositionAndWait(place_pos_mm, orientation_deg)) {
        qWarning() << "[ROBOT_SEQ] Step 2 (Place) FAILED. Aborting sequence.";
        return;
    }

    // Step 3: 그리퍼 열기 (컵 놓기)
    qDebug() << "[ROBOT] Step 3/4: Opening Gripper (Release)";
    m_robotController->onGripperAction(0); // 0 = Open
    QThread::msleep(1500);

    // Step 4: 후퇴 위치로 이동
    qDebug() << "[ROBOT] Step 4/4: Moving to Retreat Pose";
    if (!m_robotController->moveToPositionAndWait(retreat_pos_mm, orientation_deg)) {
        qWarning() << "[ROBOT_SEQ] Step 4 (Retreat) FAILED.";
    }

    qInfo() << "[ROBOT_SEQ] ========== Align-Hang Sequence Completed! ==========";
}
