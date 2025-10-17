#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QLabel>
#include <QThread>
#include <QtMath>

// 전역 변수 및 콜백 함수
// ----------------------------------------------------
using namespace DRAFramework;

CDRFLEx GlobalDrfl;
bool g_bHasControlAuthority = false;
bool g_bServoOnAttempted = false;
bool g_TpInitailizingComplted = false;
QLabel* MainWindow::s_robotStateLabel = nullptr;
MainWindow* MainWindow::s_instance = nullptr;

void OnMonitoringStateCB(const ROBOT_STATE eState) {
    if (!g_bHasControlAuthority) {
        return;
    }
    switch (eState) {
    case STATE_SAFE_STOP:
        qDebug() << "[ROBOT] In STATE_SAFE_STOP, resetting safe stop.";
        GlobalDrfl.SetSafeStopResetType(SAFE_STOP_RESET_TYPE_DEFAULT);
        GlobalDrfl.SetRobotControl(CONTROL_RESET_SAFET_STOP);
        break;
    case STATE_SAFE_STOP2:
        qDebug() << "[ROBOT] In STATE_SAFE_STOP2, recovering safe stop.";
        GlobalDrfl.SetRobotControl(CONTROL_RECOVERY_SAFE_STOP);
        break;
    case STATE_SAFE_OFF2:
        qDebug() << "[ROBOT] In STATE_SAFE_OFF2, recovering safe off.";
        GlobalDrfl.SetRobotControl(CONTROL_RECOVERY_SAFE_OFF);
        break;
    case STATE_SAFE_OFF:
        qDebug() << "[ROBOT] In STATE_SAFE_OFF, attempting to turn servo ON.";
        GlobalDrfl.SetRobotControl(CONTROL_SERVO_ON);
        break;
    case STATE_STANDBY:
        if (!g_bServoOnAttempted) {
            qDebug() << "[ROBOT] In STATE_STANDBY, attempting to turn servo ON for the first time.";
            if (GlobalDrfl.SetRobotControl(CONTROL_SERVO_ON)) {
                g_bServoOnAttempted = true;
            }
        }
        break;
    default:
        break;
    }
}

void OnTpInitializingCompleted() {
    g_TpInitailizingComplted = true;
    GlobalDrfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
}

void OnDisConnected() {
    g_bServoOnAttempted = false;
    g_bHasControlAuthority = false;
    g_TpInitailizingComplted = false;
    if (MainWindow::s_robotStateLabel) {
        MainWindow::s_robotStateLabel->setText("Robot Status: DISCONNECTED");
    }
    // Init에서 그리퍼 코드를 제거했으므로 플래그만 리셋
    if (MainWindow::s_instance) {
        MainWindow::s_instance->m_isGripperOpenPending = false;
    }
}

void OnMonitroingAccessControlCB(const MONITORING_ACCESS_CONTROL eTrasnsitControl) {
    switch (eTrasnsitControl) {
    case MONITORING_ACCESS_CONTROL_REQUEST:
        GlobalDrfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_RESPONSE_YES);
        break;
    case MONITORING_ACCESS_CONTROL_GRANT:
        qDebug() << "[ROBOT] Control Authority Granted.";
        g_bHasControlAuthority = true;
        OnMonitoringStateCB(GlobalDrfl.GetRobotState());

        // Init 시 그리퍼 열기 로직 제거.
        if (MainWindow::s_instance && MainWindow::s_instance->m_isGripperOpenPending) {
            MainWindow::s_instance->m_isGripperOpenPending = false; // 플래그 리셋만 수행
        }

        break;
    case MONITORING_ACCESS_CONTROL_DENY:
    case MONITORING_ACCESS_CONTROL_LOSS:
        qDebug() << "[ROBOT] Control Authority Lost or Denied.";
        g_bHasControlAuthority = false;
        g_bServoOnAttempted = false;
        if (g_TpInitailizingComplted) {
            GlobalDrfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
        }
        if (MainWindow::s_instance) {
            MainWindow::s_instance->m_isGripperOpenPending = false;
        }
        break;
    default:
        break;
    }
}
// ----------------------------------------------------

// MainWindow 클래스 멤버 함수 구현
// ----------------------------------------------------

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , m_isWaitingForMoveCompletion(false)
    , m_isGripperOpenPending(false)
{
    ui->setupUi(this);
    s_robotStateLabel = ui->RobotState;
    MainWindow::s_instance = this;
    s_robotStateLabel->setText("Robot Status: Disconnected");

    connect(ui->CaptureButton, &QPushButton::clicked,
            ui->widget, &RealSenseWidget::captureAndProcess);
    connect(ui->ResetPosButton, &QPushButton::clicked, this, &MainWindow::on_ResetPosButton_clicked);

    // ✨ [추가] 그리퍼 버튼 연결
    connect(ui->GripperOpenButton, &QPushButton::clicked, this, &MainWindow::on_GripperOpenButton_clicked);
    connect(ui->GripperCloseButton, &QPushButton::clicked, this, &MainWindow::on_GripperCloseButton_clicked);

    m_monitorThread = new QThread(this);
    m_robotMonitor = new RobotMonitor();
    m_robotMonitor->moveToThread(m_monitorThread);

    connect(m_monitorThread, &QThread::started, m_robotMonitor, &RobotMonitor::startMonitoring);
    connect(m_robotMonitor, &RobotMonitor::robotStateChanged, this, &MainWindow::updateRobotStateLabel);
    connect(m_robotMonitor, &RobotMonitor::robotPoseUpdated, this, &MainWindow::updateRobotPoseLabel);
    connect(m_monitorThread, &QThread::finished, m_robotMonitor, &QObject::deleteLater);

    connect(m_robotMonitor, &RobotMonitor::robotTransformUpdated,
            ui->widget, &RealSenseWidget::onRobotTransformUpdated);

    // 기존 moveL 요청 연결
    connect(ui->widget, &RealSenseWidget::requestRobotMove,
            this, &MainWindow::onMoveRobot);

    // ✨ [추가] 그리퍼 제어 시그널 연결 (M 키, 버튼 등에서 호출)
    connect(ui->widget, &RealSenseWidget::requestGripperAction,
            this, &MainWindow::onGripperAction);

    // ✨ [추가] Pick & Return 요청 연결 (D 키 시퀀스)
    connect(ui->widget, &RealSenseWidget::requestRobotPickAndReturn,
            this, &MainWindow::onRobotPickAndReturn);

    ui->widget->setShowPlot(true);
}

MainWindow::~MainWindow()
{
    if (m_monitorThread && m_monitorThread->isRunning()) {
        m_monitorThread->quit();
        m_monitorThread->wait(1000);
    }
    GlobalDrfl.CloseConnection();
    delete ui;
}

void MainWindow::on_RobotInit_clicked()
{
    const char* robot_ip = "192.168.137.100";
    g_bServoOnAttempted = false;
    g_bHasControlAuthority = false;
    g_TpInitailizingComplted = false;
    m_isGripperOpenPending = false; // Init 버튼에서 그리퍼 요청 플래그 제거

    GlobalDrfl.set_on_tp_initializing_completed(OnTpInitializingCompleted);
    GlobalDrfl.set_on_disconnected(OnDisConnected);
    GlobalDrfl.set_on_monitoring_access_control(OnMonitroingAccessControlCB);
    GlobalDrfl.set_on_monitoring_state(OnMonitoringStateCB);

    if (GlobalDrfl.open_connection(robot_ip)) {
        s_robotStateLabel->setText("Robot Status: CONNECTING...");
        SYSTEM_VERSION tSysVerion{};
        GlobalDrfl.get_system_version(&tSysVerion);
        GlobalDrfl.setup_monitoring_version(1);
        GlobalDrfl.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
        GlobalDrfl.set_robot_system(ROBOT_SYSTEM_REAL);
        GlobalDrfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
        if (!m_monitorThread->isRunning()) {
            m_monitorThread->start();
        }
    } else {
        s_robotStateLabel->setText("Robot Status: FAILED TO CONNECT");
    }
}

// ====================================================================
// ✨ [추가/수정] 그리퍼 제어 통합 슬롯 구현
// ====================================================================
void MainWindow::onGripperAction(int action)
{
    if (!g_bHasControlAuthority) {
        qWarning() << "[GRIPPER] Cannot perform action: No control authority (Ctrl X).";
        return;
    }
    if (GlobalDrfl.GetRobotState() != STATE_STANDBY) {
        qWarning() << "[GRIPPER] Cannot perform action: Robot is not in STANDBY state.";
        return;
    }

    // GRIPPER_PIN_A (0) = GPIO_TOOL_DIGITAL_INDEX_1
    // GRIPPER_PIN_B (1) = GPIO_TOOL_DIGITAL_INDEX_2

    qDebug() << "[GRIPPER] Executing sequence: " << (action == 0 ? "OPEN" : "CLOSE");

    // 1. A=0, B=0 (리셋/시작)
    GlobalDrfl.set_tool_digital_output(GPIO_TOOL_DIGITAL_INDEX_1, false);
    GlobalDrfl.set_tool_digital_output(GPIO_TOOL_DIGITAL_INDEX_2, false);
    QThread::msleep(500); // 딜레이 (0.5초)

    if (action == 0) { // Open
        // 2. A=1, B=0 (열기 명령)
        GlobalDrfl.set_tool_digital_output(GPIO_TOOL_DIGITAL_INDEX_1, true);
        GlobalDrfl.set_tool_digital_output(GPIO_TOOL_DIGITAL_INDEX_2, false);
        qDebug() << "[GRIPPER] Open command sent.";
    } else { // Close
        // 2. A=0, B=1 (닫기 명령)
        GlobalDrfl.set_tool_digital_output(GPIO_TOOL_DIGITAL_INDEX_1, false);
        GlobalDrfl.set_tool_digital_output(GPIO_TOOL_DIGITAL_INDEX_2, true);
        qDebug() << "[GRIPPER] Close command sent.";
    }
    QThread::msleep(500); // 딜레이 (0.5초)
}

void MainWindow::on_GripperOpenButton_clicked()
{
    onGripperAction(0); // 0: Open
}

void MainWindow::on_GripperCloseButton_clicked()
{
    onGripperAction(1); // 1: Close
}
// ====================================================================


void MainWindow::onMoveRobot(const QVector3D& position_mm, const QVector3D& orientation_deg)
{
    if (!g_bHasControlAuthority) {
        qWarning() << "[ROBOT] Cannot move: No control authority.";
        return;
    }

    if (GlobalDrfl.GetRobotState() != STATE_STANDBY) {
        qWarning() << "[ROBOT] Cannot move: Robot is not in STANDBY state.";
        return;
    }

    qInfo() << "[ROBOT] Received move request. Executing movel command.";

    // 전달받은 위치(mm)와 각도(deg)를 로봇 명령 배열에 직접 할당합니다.
    float target_posx[6];
    target_posx[0] = position_mm.x();
    target_posx[1] = position_mm.y();
    target_posx[2] = position_mm.z();
    target_posx[3] = orientation_deg.x(); // A (Rx)
    target_posx[4] = orientation_deg.y(); // B (Ry)
    target_posx[5] = orientation_deg.z(); // C (Rz)

    float velx[2] = {100.0f, 60.0f};
    float accx[2] = {200.0f, 120.0f};

    qDebug() << "[ROBOT] Moving to Pos(mm):" << target_posx[0] << target_posx[1] << target_posx[2]
             << "Rot(deg):" << target_posx[3] << target_posx[4] << target_posx[5];

    m_isWaitingForMoveCompletion = true;
    GlobalDrfl.movel(target_posx, velx, accx);
}

// ✨ [추가] 'D' 키: 목표 위치로 하강, 잡기, 복귀 동작 실행 슬롯 구현
void MainWindow::onRobotPickAndReturn(const QVector3D& target_pos_mm, const QVector3D& target_ori_deg, const QVector3D& approach_pos_mm, const QVector3D& approach_ori_deg)
{
    if (!g_bHasControlAuthority) {
        qWarning() << "[PICK] Cannot start sequence: No control authority.";
        return;
    }
    if (GlobalDrfl.GetRobotState() != STATE_STANDBY) {
        qWarning() << "[PICK] Cannot start sequence: Robot is not in STANDBY state.";
        return;
    }

    qInfo() << "[PICK] Starting pick sequence (D key).";

    // 하강/복귀 속도/가속도 (파이썬 코드 참고: vel=80, acc=80)
    float velx_down[2] = {80.0f, 40.0f};
    float accx_down[2] = {80.0f, 40.0f};

    // 1. 목표 위치로 하강
    qDebug() << "   - 1. Moving down to final target position...";
    float final_posx[6];
    final_posx[0] = target_pos_mm.x(); final_posx[1] = target_pos_mm.y(); final_posx[2] = target_pos_mm.z();
    final_posx[3] = target_ori_deg.x(); final_posx[4] = target_ori_deg.y(); final_posx[5] = target_ori_deg.z();

    // 동기적 이동을 위해 movel 호출 후 상태가 STANDBY로 돌아올 때까지 대기합니다.
    m_isWaitingForMoveCompletion = true;
    if (GlobalDrfl.movel(final_posx, velx_down, accx_down)) {
        QThread::msleep(1500);

        // 2. 그리퍼 닫기 (잡기)
        qDebug() << "   - 2. Closing gripper (Pick action)...";
        onGripperAction(1); // 1: Close
        QThread::msleep(1000); // 1초 대기 (파이썬 코드 참고)

        // 3. 접근 위치로 복귀 (move up)
        qDebug() << "   - 3. Moving back up to approach position...";
        float approach_posx[6];
        approach_posx[0] = approach_pos_mm.x(); approach_posx[1] = approach_pos_mm.y(); approach_posx[2] = approach_pos_mm.z();
        approach_posx[3] = approach_ori_deg.x(); approach_posx[4] = approach_ori_deg.y(); approach_posx[5] = approach_ori_deg.z();

        m_isWaitingForMoveCompletion = true;
        if (GlobalDrfl.movel(approach_posx, velx_down, accx_down)) {
            QThread::msleep(1500); // 복귀 대기
            qInfo() << "[PICK] Pick sequence completed. Robot is at approach pose.";
        } else {
            qWarning() << "[PICK] ERROR: Move back to approach failed.";
        }
    } else {
        qWarning() << "[PICK] ERROR: Move down to target failed.";
    }
    m_isWaitingForMoveCompletion = false;
}


void MainWindow::updateRobotStateLabel(int state)
{
    ROBOT_STATE eState = (ROBOT_STATE)state;

    if (m_isWaitingForMoveCompletion && eState == STATE_STANDBY) {
        m_isWaitingForMoveCompletion = false;

        LPROBOT_TASK_POSE current_pose = GlobalDrfl.get_current_posx();
        if (current_pose) {
            qDebug() << "[ROBOT MOVE END] Final End-Effector Orientation (A, B, C deg):"
                     << current_pose->_fTargetPos[3]
                     << current_pose->_fTargetPos[4]
                     << current_pose->_fTargetPos[5];
        } else {
            qWarning() << "[ROBOT MOVE END] Could not retrieve final pose.";
        }
    }

    if (s_robotStateLabel) {
        QString stateText;
        QString controlStatus = g_bHasControlAuthority ? " [Ctrl O]" : " [Ctrl X]";
        switch (eState) {
        case STATE_NOT_READY:        stateText = "Not Ready"; break;
        case STATE_INITIALIZING:     stateText = "Initializing"; break;
        case STATE_STANDBY:          stateText = "✅ Standby"; break;
        case STATE_MOVING:           stateText = "Moving"; break;
        case STATE_EMERGENCY_STOP:   stateText = "🚨 E-Stop"; break;
        case STATE_SAFE_STOP:        stateText = "⚠️ Safe Stop"; break;
        case STATE_SAFE_OFF:         stateText = "⚠️ Safe Off"; break;
        case STATE_SAFE_STOP2:       stateText = "⚠️ Safe Stop 2"; break;
        case STATE_SAFE_OFF2:        stateText = "Safe Off 2"; break;
        case STATE_RECOVERY:         stateText = "Recovery"; break;
        case STATE_TEACHING:         stateText = "Teaching"; break;
        default:                     stateText = QString("Unknown (%1)").arg(eState); break;
        }
        s_robotStateLabel->setText("Status: " + stateText + controlStatus);
        s_robotStateLabel->adjustSize();
    }
}
void MainWindow::on_ResetPosButton_clicked()
{
    if (!g_bHasControlAuthority) {
        qWarning() << "[ROBOT] Cannot move: No control authority.";
        return;
    }

    if (GlobalDrfl.GetRobotState() != STATE_STANDBY) {
        qWarning() << "[ROBOT] Cannot move: Robot is not in STANDBY state.";
        return;
    }

    qInfo() << "[ROBOT] Moving to Reset Position.";

    float target_posx[6] = {349.0f, 9.21f, 378.46f, 172.0f, -139.0f, -179.0f};
    float velx[2] = {150.0f, 90.0f};
    float accx[2] = {300.0f, 180.0f};

    qDebug() << "[ROBOT] Moving to Pos(mm):" << target_posx[0] << target_posx[1] << target_posx[2]
             << "Rot(deg):" << target_posx[3] << target_posx[4] << target_posx[5];

    m_isWaitingForMoveCompletion = true;
    GlobalDrfl.movel(target_posx, velx, accx);
}

void MainWindow::updateRobotPoseLabel(const float* pose)
{
    if (ui->RobotPos && pose) {
        QString pose_str = QString("Pose: X:%1 Y:%2 Z:%3 | A:%4 B:%5 C:%6")
        .arg(pose[0], 0, 'f', 1)
            .arg(pose[1], 0, 'f', 1)
            .arg(pose[2], 0, 'f', 1)
            .arg(pose[3], 0, 'f', 1)
            .arg(pose[4], 0, 'f', 1)
            .arg(pose[5], 0, 'f', 1);
        ui->RobotPos->setText(pose_str);
    }
}
