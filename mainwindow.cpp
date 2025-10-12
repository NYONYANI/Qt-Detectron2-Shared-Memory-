#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QLabel>
#include <QThread>
#include <QtMath>

// 전역 변수 및 콜백 함수 (변경 없음, 이전과 동일)
// ----------------------------------------------------
using namespace DRAFramework;

CDRFLEx GlobalDrfl;
bool g_bHasControlAuthority = false;
bool g_bServoOnAttempted = false;
bool g_TpInitailizingComplted = false;
QLabel* MainWindow::s_robotStateLabel = nullptr;

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
        break;
    case MONITORING_ACCESS_CONTROL_DENY:
    case MONITORING_ACCESS_CONTROL_LOSS:
        qDebug() << "[ROBOT] Control Authority Lost or Denied.";
        g_bHasControlAuthority = false;
        g_bServoOnAttempted = false;
        if (g_TpInitailizingComplted) {
            GlobalDrfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
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
{
    ui->setupUi(this);
    s_robotStateLabel = ui->RobotState;
    s_robotStateLabel->setText("Robot Status: Disconnected");

    connect(ui->CaptureButton, &QPushButton::clicked,
            ui->widget, &RealSenseWidget::captureAndProcess);
    connect(ui->ResetPosButton, &QPushButton::clicked, this, &MainWindow::on_ResetPosButton_clicked);
    m_monitorThread = new QThread(this);
    m_robotMonitor = new RobotMonitor();
    m_robotMonitor->moveToThread(m_monitorThread);

    connect(m_monitorThread, &QThread::started, m_robotMonitor, &RobotMonitor::startMonitoring);
    connect(m_robotMonitor, &RobotMonitor::robotStateChanged, this, &MainWindow::updateRobotStateLabel);
    connect(m_robotMonitor, &RobotMonitor::robotPoseUpdated, this, &MainWindow::updateRobotPoseLabel);
    connect(m_monitorThread, &QThread::finished, m_robotMonitor, &QObject::deleteLater);

    connect(m_robotMonitor, &RobotMonitor::robotTransformUpdated,
            ui->widget, &RealSenseWidget::onRobotTransformUpdated);

    // ✨ [수정] 변경된 시그널/슬롯에 맞게 connect 구문 업데이트
    connect(ui->widget, &RealSenseWidget::requestRobotMove,
            this, &MainWindow::onMoveRobot);

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

// ================== ✨ [수정된 함수 시작] ✨ ==================
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

    GlobalDrfl.movel(target_posx, velx, accx);
}
// ================== ✨ [수정된 함수 끝] ✨ ====================


void MainWindow::updateRobotStateLabel(int state)
{
    ROBOT_STATE eState = (ROBOT_STATE)state;
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

    // 요청하신 좌표 (X, Y, Z, A, B, C)
    float target_posx[6] = {349.0f, 9.21f, 378.46f, 172.0f, -139.0f, -179.0f};

    // 이동 속도 및 가속도 설정
    float velx[2] = {150.0f, 90.0f}; // 선속도, 각속도
    float accx[2] = {300.0f, 180.0f}; // 선가속도, 각가속도

    qDebug() << "[ROBOT] Moving to Pos(mm):" << target_posx[0] << target_posx[1] << target_posx[2]
             << "Rot(deg):" << target_posx[3] << target_posx[4] << target_posx[5];

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
