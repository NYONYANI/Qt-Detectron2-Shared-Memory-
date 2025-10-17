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
    case STATE_STANDBY:
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

    if (MainWindow::s_instance) {
        MainWindow::s_instance->updateUiForState(MainWindow::RobotConnectionState::Disconnected);
        if (MainWindow::s_robotStateLabel) {
            MainWindow::s_robotStateLabel->setText("Robot Status: DISCONNECTED");
        }
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
        if (MainWindow::s_instance) {
            MainWindow::s_instance->updateUiForState(MainWindow::RobotConnectionState::Connected);
            if (MainWindow::s_instance->m_isGripperOpenPending) {
                MainWindow::s_instance->m_isGripperOpenPending = false;
            }
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
            MainWindow::s_instance->updateUiForState(MainWindow::RobotConnectionState::Disconnected);
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
    , m_robotConnectionState(RobotConnectionState::Disconnected) // ✨ [수정] 상태 초기화
{
    ui->setupUi(this);
    s_robotStateLabel = ui->RobotState;
    MainWindow::s_instance = this;
    s_robotStateLabel->setText("Robot Status: Disconnected");

    connect(ui->CaptureButton, &QPushButton::clicked,
            ui->widget, &RealSenseWidget::captureAndProcess);
    connect(ui->ResetPosButton, &QPushButton::clicked, this, &MainWindow::on_ResetPosButton_clicked);
    connect(ui->GripperOpenButton, &QPushButton::clicked, this, &MainWindow::on_GripperOpenButton_clicked);
    connect(ui->GripperCloseButton, &QPushButton::clicked, this, &MainWindow::on_GripperCloseButton_clicked);
    connect(ui->MoveButton, &QPushButton::clicked, this, &MainWindow::on_MoveButton_clicked);

    m_monitorThread = new QThread(this);
    m_robotMonitor = new RobotMonitor();
    m_robotMonitor->moveToThread(m_monitorThread);

    connect(m_monitorThread, &QThread::started, m_robotMonitor, &RobotMonitor::startMonitoring);
    connect(m_robotMonitor, &RobotMonitor::robotStateChanged, this, &MainWindow::updateRobotStateLabel);
    connect(m_robotMonitor, &RobotMonitor::robotPoseUpdated, this, &MainWindow::updateRobotPoseLabel);
    connect(m_monitorThread, &QThread::finished, m_robotMonitor, &QObject::deleteLater);
    connect(m_robotMonitor, &RobotMonitor::robotTransformUpdated,
            ui->widget, &RealSenseWidget::onRobotTransformUpdated);
    connect(ui->widget, &RealSenseWidget::requestRobotMove,
            this, &MainWindow::onMoveRobot);
    connect(ui->widget, &RealSenseWidget::requestGripperAction,
            this, &MainWindow::onGripperAction);
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

// ✨ [추가] 현재 연결 상태를 반환하는 getter 함수
MainWindow::RobotConnectionState MainWindow::getConnectionState() const
{
    return m_robotConnectionState;
}

// ✨ [수정] 로봇 상태에 따라 버튼의 텍스트와 상태를 변경하는 함수
void MainWindow::updateUiForState(RobotConnectionState state)
{
    m_robotConnectionState = state;
    switch (state) {
    case RobotConnectionState::Disconnected:
        ui->RobotInit->setText("Init");
        ui->RobotInit->setEnabled(true);
        break;
    case RobotConnectionState::Connected:
        ui->RobotInit->setText("Servo ON");
        ui->RobotInit->setEnabled(true);
        break;
    case RobotConnectionState::ServoOn:
        ui->RobotInit->setText("Close");
        ui->RobotInit->setEnabled(true);
        break;
    }
}

// ✨ [수정] RobotInit 버튼을 상태에 따라 다르게 동작하도록 수정
void MainWindow::on_RobotInit_clicked()
{
    switch (m_robotConnectionState)
    {
    // 1단계: "Init" 버튼 클릭 시 -> 로봇 연결
    case RobotConnectionState::Disconnected:
    {
        const char* robot_ip = "192.168.137.100";
        g_bServoOnAttempted = false;
        g_bHasControlAuthority = false;
        g_TpInitailizingComplted = false;
        m_isGripperOpenPending = false;

        GlobalDrfl.set_on_tp_initializing_completed(OnTpInitializingCompleted);
        GlobalDrfl.set_on_disconnected(OnDisConnected);
        GlobalDrfl.set_on_monitoring_access_control(OnMonitroingAccessControlCB);
        GlobalDrfl.set_on_monitoring_state(OnMonitoringStateCB);

        if (GlobalDrfl.open_connection(robot_ip)) {
            s_robotStateLabel->setText("Robot Status: CONNECTING...");
            ui->RobotInit->setEnabled(false);
            ui->RobotInit->setText("Connecting...");

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
            updateUiForState(RobotConnectionState::Disconnected);
        }
    }
    break;

    // 2단계: "Servo ON" 버튼 클릭 시 -> 서보 켜기
    case RobotConnectionState::Connected:
    {
        qDebug() << "[ROBOT] 'Servo ON' button clicked.";
        if (g_bHasControlAuthority && !g_bServoOnAttempted) {
            ROBOT_STATE currentState = GlobalDrfl.GetRobotState();
            if (currentState == STATE_SAFE_OFF || currentState == STATE_STANDBY) {
                if (GlobalDrfl.SetRobotControl(CONTROL_SERVO_ON)) {
                    qInfo() << "[ROBOT] Servo ON command sent successfully.";
                    g_bServoOnAttempted = true; // 서보 온 시도 플래그 설정
                } else {
                    qWarning() << "[ROBOT] Failed to send Servo ON command.";
                }
            } else {
                qWarning() << "[ROBOT] Cannot turn servo on, robot is not in a valid state. Current state:" << currentState;
            }
        } else if (g_bServoOnAttempted) {
            qDebug() << "[ROBOT] Servo ON command was already sent.";
        } else {
            qWarning() << "[ROBOT] Cannot turn servo on, no control authority.";
        }
    }
    break;

    // 3단계: "Close" 버튼 클릭 시 -> 연결 해제
    case RobotConnectionState::ServoOn:
        qInfo() << "[ROBOT] 'Close' button clicked. Disconnecting from robot.";
        GlobalDrfl.CloseConnection();
        // OnDisConnected 콜백이 호출되면서 UI 상태가 Disconnected로 변경됩니다.
        break;
    }
}


// ====================================================================
// 그리퍼 제어 및 로봇 이동 함수 (이하 변경 없음)
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
    qDebug() << "[GRIPPER] Executing sequence: " << (action == 0 ? "OPEN" : "CLOSE");
    GlobalDrfl.set_tool_digital_output(GPIO_TOOL_DIGITAL_INDEX_1, false);
    GlobalDrfl.set_tool_digital_output(GPIO_TOOL_DIGITAL_INDEX_2, false);
    QThread::msleep(500);
    if (action == 0) {
        GlobalDrfl.set_tool_digital_output(GPIO_TOOL_DIGITAL_INDEX_1, true);
        GlobalDrfl.set_tool_digital_output(GPIO_TOOL_DIGITAL_INDEX_2, false);
        qDebug() << "[GRIPPER] Open command sent.";
    } else {
        GlobalDrfl.set_tool_digital_output(GPIO_TOOL_DIGITAL_INDEX_1, false);
        GlobalDrfl.set_tool_digital_output(GPIO_TOOL_DIGITAL_INDEX_2, true);
        qDebug() << "[GRIPPER] Close command sent.";
    }
    QThread::msleep(500);
}

void MainWindow::on_GripperOpenButton_clicked() { onGripperAction(0); }
void MainWindow::on_GripperCloseButton_clicked() { onGripperAction(1); }

void MainWindow::onMoveRobot(const QVector3D& position_mm, const QVector3D& orientation_deg)
{
    if (!g_bHasControlAuthority || GlobalDrfl.GetRobotState() != STATE_STANDBY) {
        qWarning() << "[ROBOT] Cannot move: No control authority or not in STANDBY.";
        return;
    }
    qInfo() << "[ROBOT] Received move request. Executing movel command.";
    float target_posx[6];
    target_posx[0] = position_mm.x();
    target_posx[1] = position_mm.y();
    target_posx[2] = position_mm.z();
    target_posx[3] = orientation_deg.x();
    target_posx[4] = orientation_deg.y();
    target_posx[5] = orientation_deg.z();
    float velx[2] = {100.0f, 60.0f};
    float accx[2] = {200.0f, 120.0f};
    qDebug() << "[ROBOT] Moving to Pos(mm):" << target_posx[0] << target_posx[1] << target_posx[2]
             << "Rot(deg):" << target_posx[3] << target_posx[4] << target_posx[5];
    m_isWaitingForMoveCompletion = true;
    GlobalDrfl.movel(target_posx, velx, accx);
}

void MainWindow::onRobotPickAndReturn(const QVector3D& target_pos_mm, const QVector3D& target_ori_deg, const QVector3D& approach_pos_mm, const QVector3D& approach_ori_deg)
{
    if (!g_bHasControlAuthority || GlobalDrfl.GetRobotState() != STATE_STANDBY) {
        qWarning() << "[PICK] Cannot start sequence: No control authority or not in STANDBY.";
        return;
    }
    qInfo() << "[PICK] Starting pick sequence (D key).";
    float velx_down[2] = {80.0f, 40.0f};
    float accx_down[2] = {80.0f, 40.0f};
    float final_posx[6];
    final_posx[0] = target_pos_mm.x(); final_posx[1] = target_pos_mm.y(); final_posx[2] = target_pos_mm.z();
    final_posx[3] = target_ori_deg.x(); final_posx[4] = target_ori_deg.y(); final_posx[5] = target_ori_deg.z();
    m_isWaitingForMoveCompletion = true;
    if (GlobalDrfl.movel(final_posx, velx_down, accx_down)) {
        QThread::msleep(1500);
        qDebug() << "   - 2. Closing gripper (Pick action)...";
        onGripperAction(1);
        QThread::msleep(1000);
        qDebug() << "   - 3. Moving back up to approach position...";
        float approach_posx[6];
        approach_posx[0] = approach_pos_mm.x(); approach_posx[1] = approach_pos_mm.y(); approach_posx[2] = approach_pos_mm.z();
        approach_posx[3] = approach_ori_deg.x(); approach_posx[4] = approach_ori_deg.y(); approach_posx[5] = approach_ori_deg.z();
        m_isWaitingForMoveCompletion = true;
        if (GlobalDrfl.movel(approach_posx, velx_down, accx_down)) {
            QThread::msleep(1500);
            qInfo() << "[PICK] Pick sequence completed. Robot is at approach pose.";
        } else { qWarning() << "[PICK] ERROR: Move back to approach failed."; }
    } else { qWarning() << "[PICK] ERROR: Move down to target failed."; }
    m_isWaitingForMoveCompletion = false;
}

void MainWindow::on_MoveButton_clicked()
{
    if (!g_bHasControlAuthority || GlobalDrfl.GetRobotState() != STATE_STANDBY) {
        qWarning() << "[ROBOT] Cannot move: No control authority or not in STANDBY.";
        return;
    }
    qInfo() << "[ROBOT] Moving to Drop-off Position.";
    float target_posx[6] = {401.0f, 0.0f, 415.0f, 139.0f, -109.0f, -165.0f};
    float velx[2] = {150.0f, 90.0f};
    float accx[2] = {300.0f, 180.0f};
    qDebug() << "[ROBOT] Moving to Pos(mm):" << target_posx[0] << target_posx[1] << target_posx[2]
             << "Rot(deg):" << target_posx[3] << target_posx[4] << target_posx[5];
    m_isWaitingForMoveCompletion = true;
    GlobalDrfl.movel(target_posx, velx, accx);
}

void MainWindow::updateRobotStateLabel(int state)
{
    ROBOT_STATE eState = (ROBOT_STATE)state;
    if (m_isWaitingForMoveCompletion && eState == STATE_STANDBY) {
        m_isWaitingForMoveCompletion = false;
        LPROBOT_TASK_POSE current_pose = GlobalDrfl.get_current_posx();
        if (current_pose) {
            qDebug() << "[ROBOT MOVE END] Final End-Effector Orientation (A, B, C deg):"
                     << current_pose->_fTargetPos[3] << current_pose->_fTargetPos[4] << current_pose->_fTargetPos[5];
        } else { qWarning() << "[ROBOT MOVE END] Could not retrieve final pose."; }
    }

    // ✨ [추가] 서보가 성공적으로 켜졌는지 확인하고 UI 상태를 ServoOn으로 변경
    if (eState == STATE_STANDBY && g_bServoOnAttempted && getConnectionState() == RobotConnectionState::Connected) {
        updateUiForState(RobotConnectionState::ServoOn);
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
    if (!g_bHasControlAuthority || GlobalDrfl.GetRobotState() != STATE_STANDBY) {
        qWarning() << "[ROBOT] Cannot move: No control authority or not in STANDBY.";
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
        .arg(pose[0], 0, 'f', 1).arg(pose[1], 0, 'f', 1).arg(pose[2], 0, 'f', 1)
            .arg(pose[3], 0, 'f', 1).arg(pose[4], 0, 'f', 1).arg(pose[5], 0, 'f', 1);
        ui->RobotPos->setText(pose_str);
    }
}
