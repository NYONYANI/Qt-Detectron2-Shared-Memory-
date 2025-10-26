#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QLabel>
#include <QThread>
#include <QtMath>
#include <QTimer>
// QApplication은 on_MoveButton_clicked에서 제거되었으므로 여기서 필요 없습니다.

// 전역 변수 및 콜백 함수
using namespace DRAFramework;

CDRFLEx GlobalDrfl;
bool g_bHasControlAuthority = false;
bool g_bServoOnAttempted = false;
bool g_TpInitailizingComplted = false;
QLabel* MainWindow::s_robotStateLabel = nullptr;
MainWindow* MainWindow::s_instance = nullptr;

void OnMonitoringStateCB(const ROBOT_STATE eState) {
    if (!g_bHasControlAuthority) { return; }
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
    default: break;
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
        if (MainWindow::s_instance->getConnectionState() == MainWindow::RobotConnectionState::Connecting) {
            qDebug() << "[ROBOT] Disconnected event received while connecting. Ignoring for UI stability.";
            return;
        }
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
            if (MainWindow::s_instance->getConnectionState() == MainWindow::RobotConnectionState::Connecting) {
                qDebug() << "[ROBOT] Loss/Deny event received while connecting. Ignoring for UI stability.";
                return;
            }
            MainWindow::s_instance->updateUiForState(MainWindow::RobotConnectionState::Disconnected);
            MainWindow::s_instance->m_isGripperOpenPending = false;
        }
        break;
    default:
        break;
    }
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , m_isGripperOpenPending(false)
    , m_robotConnectionState(RobotConnectionState::Disconnected)
{
    ui->setupUi(this);
    s_robotStateLabel = ui->RobotState;
    MainWindow::s_instance = this;
    s_robotStateLabel->setText("Robot Status: Disconnected");

    // 로봇 컨트롤러 스레드 설정
    m_robotController = new RobotController();
    m_robotController->moveToThread(&m_robotControllerThread);
    connect(&m_robotControllerThread, &QThread::finished, m_robotController, &QObject::deleteLater);

    // MainWindow -> RobotController (명령)
    connect(this, &MainWindow::requestMoveRobot, m_robotController, &RobotController::onMoveRobot);
    connect(this, &MainWindow::requestRobotPickAndReturn, m_robotController, &RobotController::onRobotPickAndReturn); // 이름 확인
    connect(this, &MainWindow::requestResetPosition, m_robotController, &RobotController::onResetPosition);
    connect(this, &MainWindow::requestGripperAction, m_robotController, &RobotController::onGripperAction);
    connect(this, &MainWindow::startRobotMonitoring, m_robotController, &RobotController::startMonitoring);

    // RobotController -> MainWindow (상태 업데이트)
    connect(m_robotController, &RobotController::robotStateChanged, this, &MainWindow::updateRobotStateLabel);
    connect(m_robotController, &RobotController::robotPoseUpdated, this, &MainWindow::updateRobotPoseLabel);
    connect(m_robotController, &RobotController::robotTransformUpdated, ui->widget, &RealSenseWidget::onRobotTransformUpdated);

    m_robotControllerThread.start();

    // UI 위젯 시그널 연결
    connect(ui->CaptureButton, &QPushButton::clicked, ui->widget, &RealSenseWidget::captureAndProcess);
    connect(ui->ResetPosButton, &QPushButton::clicked, this, &MainWindow::on_ResetPosButton_clicked);
    connect(ui->GripperOpenButton, &QPushButton::clicked, this, &MainWindow::on_GripperOpenButton_clicked);
    connect(ui->GripperCloseButton, &QPushButton::clicked, this, &MainWindow::on_GripperCloseButton_clicked);
    // (MoveButton, HandlePlotButton, MoveViewButton, MovepointButton은 on_..._clicked() 슬롯으로 자동 연결됩니다)

    // RealSenseWidget -> MainWindow (수동 시그널 전달용)
    connect(ui->widget, &RealSenseWidget::requestRobotMove, this, &MainWindow::requestMoveRobot);
    connect(ui->widget, &RealSenseWidget::requestGripperAction, this, &MainWindow::requestGripperAction);

    // ✨ [오류 발생했던 지점] 이제 mainwindow.h의 시그널 이름과 일치합니다.
    connect(ui->widget, &RealSenseWidget::requestRobotPickAndReturn, this, &MainWindow::requestRobotPickAndReturn);

    connect(ui->widget, &RealSenseWidget::requestLiftRotatePlaceSequence,
            this, &MainWindow::requestLiftRotatePlaceSequence);

    // MainWindow -> RobotController (수동 시그널 전달용)
    connect(this, &MainWindow::requestLiftRotatePlaceSequence,
            m_robotController, &RobotController::onLiftRotatePlaceSequence);

    // ✨ [추가] RealSenseWidget의 새 전체 시퀀스 시그널을 RobotController의 새 슬롯에 연결
    connect(ui->widget, &RealSenseWidget::requestFullPickAndPlaceSequence,
            m_robotController, &RobotController::onFullPickAndPlaceSequence);

    ui->widget->setShowPlot(true);
}

MainWindow::~MainWindow()
{
    m_robotControllerThread.quit();
    m_robotControllerThread.wait();
    GlobalDrfl.CloseConnection();
    delete ui;
}

void MainWindow::showEvent(QShowEvent *event)
{
    QMainWindow::showEvent(event);
    // 윈도우가 완전히 표시된 후 카메라 스트림 시작
    QTimer::singleShot(100, ui->widget, &RealSenseWidget::startCameraStream);
}


void MainWindow::on_RobotInit_clicked()
{
    switch (m_robotConnectionState)
    {
    case RobotConnectionState::Disconnected:
    {
        updateUiForState(RobotConnectionState::Connecting);
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

            emit startRobotMonitoring();
        } else {
            s_robotStateLabel->setText("Robot Status: FAILED TO CONNECT");
            updateUiForState(RobotConnectionState::Disconnected);
        }
    }
    break;
    case RobotConnectionState::Connected:
    {
        qDebug() << "[ROBOT] 'Servo ON' button clicked.";
        if (g_bHasControlAuthority && !g_bServoOnAttempted) {
            ROBOT_STATE currentState = GlobalDrfl.GetRobotState();
            if (currentState == STATE_SAFE_OFF || currentState == STATE_STANDBY) {
                if (GlobalDrfl.SetRobotControl(CONTROL_SERVO_ON)) {
                    qInfo() << "[ROBOT] Servo ON command sent successfully.";
                    g_bServoOnAttempted = true;
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

    case RobotConnectionState::ServoOn:
        qInfo() << "[ROBOT] 'Close' button clicked. Disconnecting from robot.";
        GlobalDrfl.CloseConnection();
        break;

    case RobotConnectionState::Connecting:
        break;
    }
}


MainWindow::RobotConnectionState MainWindow::getConnectionState() const
{
    return m_robotConnectionState;
}

void MainWindow::updateUiForState(RobotConnectionState state)
{
    m_robotConnectionState = state;
    switch (state) {
    case RobotConnectionState::Disconnected:
        ui->RobotInit->setText("Init");
        ui->RobotInit->setEnabled(true);
        break;
    case RobotConnectionState::Connecting:
        ui->RobotInit->setText("Connecting...");
        ui->RobotInit->setEnabled(false);
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

void MainWindow::on_GripperOpenButton_clicked() { emit requestGripperAction(0); }
void MainWindow::on_GripperCloseButton_clicked() { emit requestGripperAction(1); }
void MainWindow::on_ResetPosButton_clicked() { emit requestResetPosition(); }

// ✨ [수정] Move 버튼 클릭 시 RealSenseWidget의 새 함수를 호출
void MainWindow::on_MoveButton_clicked()
{
    qDebug() << "[MAIN] 'Move' button clicked. Initiating full automated sequence.";
    // 1~5번(계산) + M, D, Move(로봇동작)을 모두 처리하는 함수 호출
    ui->widget->runFullAutomatedSequence();
}

// ✨ [추가] HandlePlotButton 클릭 시 RealSenseWidget의 새 함수를 호출
void MainWindow::on_HandlePlotButton_clicked()
{
    qDebug() << "[MAIN] 'Handle Plot' button clicked. Requesting handle PCA plot.";
    ui->widget->onShowHandlePlot();
}

// ✨ [수정] MoveViewButton 클릭 시 '계산' 함수 호출
void MainWindow::on_MoveViewButton_clicked()
{
    qDebug() << "[MAIN] 'Move View' button clicked. Requesting handle view pose CALCULATION.";
    ui->widget->onCalculateHandleViewPose(); // ✨ 수정된 함수 호출
}

// ✨ [수정] MovepointButton 클릭 시 '이동' 함수 호출
void MainWindow::on_MovepointButton_clicked()
{
    qDebug() << "[MAIN] 'MovepointButton' clicked. Requesting robot MOVE to handle view pose.";
    ui->widget->onMoveToCalculatedHandleViewPose(); // ✨ 수정된 함수 호출
}


void MainWindow::updateRobotStateLabel(int state)
{
    ROBOT_STATE eState = (ROBOT_STATE)state;

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

void MainWindow::updateRobotPoseLabel(const float* pose)
{
    if (ui->RobotPos && pose) {
        QString pose_str = QString("Pose: X:%1 Y:%2 Z:%3 | A:%4 B:%5 C:%6")
        .arg(pose[0], 0, 'f', 1).arg(pose[1], 0, 'f', 1).arg(pose[2], 0, 'f', 1)
            .arg(pose[3], 0, 'f', 1).arg(pose[4], 0, 'f', 1).arg(pose[5], 0, 'f', 1);
        ui->RobotPos->setText(pose_str);
    }
}
