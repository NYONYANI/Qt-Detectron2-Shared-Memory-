#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QLabel>
#include <QThread>

// ----------------------------------------------------
// ✨ [수정] 모든 전역 변수와 객체를 여기서 단 한 번만 정의합니다.
// ----------------------------------------------------
using namespace DRAFramework;

CDRFLEx GlobalDrfl;
bool g_bHasControlAuthority = false;
bool g_bServoOnAttempted = false;
bool g_TpInitailizingComplted = false;
QLabel* MainWindow::s_robotStateLabel = nullptr;

// ----------------------------------------------------
// ✨ [수정] 모든 전역 콜백 함수를 여기서 단 한 번만 정의합니다.
// ----------------------------------------------------
void OnMonitoringStateCB(const ROBOT_STATE eState) {
    if (!g_bHasControlAuthority) {
        return;
    }
    switch (eState) {
        case STATE_SAFE_STOP:
            GlobalDrfl.SetSafeStopResetType(SAFE_STOP_RESET_TYPE_DEFAULT);
            GlobalDrfl.SetRobotControl(CONTROL_RESET_SAFET_STOP);
            break;
        case STATE_SAFE_STOP2:
            GlobalDrfl.SetRobotControl(CONTROL_RECOVERY_SAFE_STOP);
            break;
        case STATE_SAFE_OFF2:
            GlobalDrfl.SetRobotControl(CONTROL_RECOVERY_SAFE_OFF);
            break;
        case STATE_SAFE_OFF:
            GlobalDrfl.SetRobotControl(CONTROL_SERVO_ON);
            break;
        case STATE_STANDBY:
            if (!g_bServoOnAttempted) {
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
            GlobalDrfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_RESPONSE_NO);
            break;
        case MONITORING_ACCESS_CONTROL_GRANT:
            g_bHasControlAuthority = true;
            OnMonitoringStateCB(GlobalDrfl.GetRobotState());
            break;
        case MONITORING_ACCESS_CONTROL_DENY:
        case MONITORING_ACCESS_CONTROL_LOSS:
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
// [MainWindow 클래스 멤버 함수 구현]
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

    m_monitorThread = new QThread(this);
    m_robotMonitor = new RobotMonitor();
    m_robotMonitor->moveToThread(m_monitorThread);

    connect(m_monitorThread, &QThread::started, m_robotMonitor, &RobotMonitor::startMonitoring);
    connect(m_robotMonitor, &RobotMonitor::robotStateChanged, this, &MainWindow::updateRobotStateLabel);
    connect(m_robotMonitor, &RobotMonitor::robotPoseUpdated, this, &MainWindow::updateRobotPoseLabel);
    connect(m_monitorThread, &QThread::finished, m_robotMonitor, &QObject::deleteLater);

    connect(m_robotMonitor, &RobotMonitor::robotPoseUpdated,
            ui->widget, &RealSenseWidget::onRobotPoseUpdated);
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
