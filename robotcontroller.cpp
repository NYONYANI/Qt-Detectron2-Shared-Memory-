#include "robotcontroller.h"
#include <QDebug>
#include <QThread>
#include <QMatrix4x4>
#include <QtMath> // qRadiansToDegrees 사용
#include <cstring> // memcpy 사용

// ✨ [추가] mainwindow.cpp에 정의된 콜백 함수들의 전방 선언
void OnMonitoringStateCB(const ROBOT_STATE eState);
void OnTpInitializingCompleted();
void OnDisConnected();
void OnMonitroingAccessControlCB(const MONITORING_ACCESS_CONTROL eTrasnsitControl);


RobotController::RobotController(QObject *parent)
    : QObject(parent)
    , m_angleDebugPrinted(false)
{
    m_timer = new QTimer(this);
    // ✨ [오류 수정] &QObject::checkRobotState -> &RobotController::checkRobotState
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

//
// ✨ [추가] MainWindow의 Init 버튼 로직 (스레드에서 실행)
//
void RobotController::onInitializeRobot()
{
    const char* robot_ip = "192.168.137.100";
    g_bServoOnAttempted = false;
    g_bHasControlAuthority = false;
    g_TpInitailizingComplted = false;

    // 콜백 함수 설정
    GlobalDrfl.set_on_tp_initializing_completed(OnTpInitializingCompleted);
    GlobalDrfl.set_on_disconnected(OnDisConnected);
    GlobalDrfl.set_on_monitoring_access_control(OnMonitroingAccessControlCB);
    GlobalDrfl.set_on_monitoring_state(OnMonitoringStateCB);

    // (블로킹될 수 있는) 연결 시도
    if (GlobalDrfl.open_connection(robot_ip)) {
        qDebug() << "[ROBOT_THREAD] Connection attempt successful, waiting for callbacks...";
        SYSTEM_VERSION tSysVerion{};
        GlobalDrfl.get_system_version(&tSysVerion);
        GlobalDrfl.setup_monitoring_version(1);
        GlobalDrfl.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
        GlobalDrfl.set_robot_system(ROBOT_SYSTEM_REAL);
        GlobalDrfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);

        startMonitoring(); // ✨ [수정] 연결 성공 시 모니터링 직접 시작
    } else {
        qWarning() << "[ROBOT_THREAD] open_connection failed.";
        emit initializationFailed("Failed to connect"); // ✨ [추가] 실패 시그널 전송
    }
}

//
// ✨ [추가] MainWindow의 Servo ON 버튼 로직 (스레드에서 실행)
//
void RobotController::onServoOn()
{
    qDebug() << "[ROBOT_THREAD] 'Servo ON' command received.";
    if (g_bHasControlAuthority && !g_bServoOnAttempted) {
        ROBOT_STATE currentState = GlobalDrfl.GetRobotState();
        if (currentState == STATE_SAFE_OFF || currentState == STATE_STANDBY) {
            if (GlobalDrfl.SetRobotControl(CONTROL_SERVO_ON)) {
                qInfo() << "[ROBOT_THREAD] Servo ON command sent successfully.";
                g_bServoOnAttempted = true;
            } else {
                qWarning() << "[ROBOT_THREAD] Failed to send Servo ON command.";
            }
        } else {
            qWarning() << "[ROBOT_THREAD] Cannot turn servo on, robot is not in a valid state. Current state:" << currentState;
        }
    } else if (g_bServoOnAttempted) {
        qDebug() << "[ROBOT_THREAD] Servo ON command was already sent.";
    } else {
        qWarning() << "[ROBOT_THREAD] Cannot turn servo on, no control authority.";
    }
}

//
// ✨ [추가] MainWindow의 Close 버튼 로직 (스레드에서 실행)
//
void RobotController::onCloseConnection()
{
    qInfo() << "[ROBOT_THREAD] 'Close' command received. Disconnecting from robot.";
    GlobalDrfl.CloseConnection();
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

//
// --- 기본 동작 함수 ---
//

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

void RobotController::onResetPosition()
{
    // ✨ [수정] 이 함수를 블로킹(동기)으로 변경
    GlobalDrfl.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
    GlobalDrfl.set_robot_system(ROBOT_SYSTEM_REAL);

    if (!g_bHasControlAuthority || GlobalDrfl.GetRobotState() != STATE_STANDBY) {
        qWarning() << "[ROBOT_THREAD] Cannot move: No control authority or not in STANDBY.";
        return;
    }
    qInfo() << "[ROBOT_THREAD] Moving to Reset Position (Blocking)...";

    QVector3D target_pos_mm(260, 0, 350);
    QVector3D target_ori_deg(0, 138, 0);

    // moveToPositionAndWait 헬퍼 함수를 재사용
    if (!moveToPositionAndWait(target_pos_mm, target_ori_deg)) {
        qWarning() << "[ROBOT_THREAD] CRITICAL: moveToPositionAndWait FAILED for RESET POSE.";
    }
    qInfo() << "[ROBOT_THREAD] Reset Position complete.";
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

//
// --- 시퀀스 헬퍼 함수 (RobotSequencer가 사용) ---
//
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


//
// --- 이하 시퀀스 함수들 (onRobotPickAndReturn 등)은 모두 robotsequencer.cpp로 이동함 ---
//
