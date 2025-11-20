#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QLabel>
#include <QThread>
#include <QtMath>
#include <QTimer>
#include <QCoreApplication>

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

    m_pythonProcess = new QProcess(this);
    connect(m_pythonProcess, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, &MainWindow::onPythonServerFinished);
    connect(m_pythonProcess, &QProcess::readyReadStandardOutput, [this](){
        qDebug() << "[PYTHON STDOUT]" << m_pythonProcess->readAllStandardOutput();
    });
    connect(m_pythonProcess, &QProcess::readyReadStandardError, [this](){
        qWarning() << "[PYTHON STDERR]" << m_pythonProcess->readAllStandardError();
    });

    m_robotController = new RobotController();
    m_robotController->moveToThread(&m_robotControllerThread);
    connect(&m_robotControllerThread, &QThread::finished, m_robotController, &QObject::deleteLater);

    m_robotSequencer = new RobotSequencer();
    m_robotSequencer->setRobotController(m_robotController);
    m_robotSequencer->moveToThread(&m_robotControllerThread);
    connect(m_robotController, &RobotController::moveFinished,
            m_robotSequencer, &RobotSequencer::onMoveTaskComplete, Qt::QueuedConnection);

    connect(this, &MainWindow::requestResetPosition, m_robotController, &RobotController::onResetPosition);
    connect(this, &MainWindow::requestGripperAction, m_robotController, &RobotController::onGripperAction);

    connect(this, &MainWindow::requestInitializeRobot, m_robotController, &RobotController::onInitializeRobot);
    connect(this, &MainWindow::requestServoOn, m_robotController, &RobotController::onServoOn);
    connect(this, &MainWindow::requestCloseConnection, m_robotController, &RobotController::onCloseConnection);

    connect(this, &MainWindow::requestRobotPickAndReturn, m_robotSequencer, &RobotSequencer::onRobotPickAndReturn);
    connect(this, &MainWindow::requestLiftRotatePlaceSequence, m_robotSequencer, &RobotSequencer::onLiftRotatePlaceSequence);

    connect(this, &MainWindow::requestFullAutomation, m_robotSequencer, &RobotSequencer::onStartFullAutomation);
    connect(m_robotSequencer, &RobotSequencer::automationFinished, this, &MainWindow::onAutomationFinished);

    connect(ui->widget, &RealSenseWidget::requestAlignHangSequence, m_robotSequencer, &RobotSequencer::onAlignHangSequence);

    connect(m_robotController, &RobotController::robotStateChanged, this, &MainWindow::updateRobotStateLabel);
    connect(m_robotController, &RobotController::robotPoseUpdated, this, &MainWindow::updateRobotPoseLabel);
    connect(m_robotController, &RobotController::robotTransformUpdated, ui->widget, &RealSenseWidget::onRobotTransformUpdated);

    connect(m_robotController, &RobotController::initializationFailed, this, &MainWindow::onRobotInitFailed);

    m_robotControllerThread.start();

    connect(ui->CaptureButton, &QPushButton::clicked, ui->widget, [=](){
        ui->widget->captureAndProcess(false);
    });

    if (ui->AlignHangButton) {
        connect(ui->AlignHangButton, &QPushButton::clicked, ui->widget, &RealSenseWidget::onAlignHangRequested);
    } else {
        qWarning() << "[SETUP] 'AlignHangButton' not found in .ui file. Please add it.";
    }

    // ✨ [추가] MoveTopButton 연결
    if (ui->MoveTopButton) {
        connect(ui->MoveTopButton, &QPushButton::clicked, ui->widget, &RealSenseWidget::onMoveToTopViewPose);
    } else {
        qWarning() << "[SETUP] 'MoveTopButton' not found in .ui file. Please add it.";
    }

    connect(ui->widget, &RealSenseWidget::requestRobotMove, m_robotController, &RobotController::moveToPositionAndWait);
    connect(ui->widget, &RealSenseWidget::requestGripperAction, this, &MainWindow::requestGripperAction);

    connect(ui->widget, &RealSenseWidget::requestRobotPickAndReturn, m_robotSequencer, &RobotSequencer::onRobotPickAndReturn);
    connect(ui->widget, &RealSenseWidget::requestLiftRotatePlaceSequence,
            m_robotSequencer, &RobotSequencer::onLiftRotatePlaceSequence);

    connect(ui->widget, &RealSenseWidget::requestFullPickAndPlaceSequence,
            m_robotSequencer, &RobotSequencer::onFullPickAndPlaceSequence);
    connect(ui->widget, &RealSenseWidget::requestApproachThenGrasp,
            m_robotSequencer, &RobotSequencer::onApproachThenGrasp);

    connect(ui->ICPButton, &QPushButton::clicked, ui->widget, &RealSenseWidget::onShowICPVisualization);

    connect(ui->TopViewButton, &QPushButton::clicked, ui->widget, &RealSenseWidget::onShowTopViewAnalysis);

    connect(m_robotSequencer, &RobotSequencer::requestVisionCapture, ui->widget, [=](){
        ui->widget->captureAndProcess(true);
    });

    connect(m_robotSequencer, &RobotSequencer::requestVisionMoveViewpoint, ui->widget, &RealSenseWidget::onCalculateHandleViewPose);
    connect(m_robotSequencer, &RobotSequencer::requestVisionHandlePlot, ui->widget, [=](){
        ui->widget->onShowHandlePlot(false);
    });
    connect(m_robotSequencer, &RobotSequencer::requestMoveToViewpoint, ui->widget, &RealSenseWidget::onMoveToCalculatedHandleViewPose);
    connect(m_robotSequencer, &RobotSequencer::requestGraspHandle, ui->widget, &RealSenseWidget::onMoveToRandomGraspPoseRequested);

    connect(m_robotSequencer, &RobotSequencer::requestToggleMask, ui->widget, &RealSenseWidget::onToggleMaskedPoints);
    connect(m_robotSequencer, &RobotSequencer::requestToggleDenoise, ui->widget, &RealSenseWidget::onDenoisingToggled);
    connect(m_robotSequencer, &RobotSequencer::requestToggleZFilter, ui->widget, &RealSenseWidget::onZFilterToggled);

    connect(ui->widget, &RealSenseWidget::visionTaskComplete, m_robotSequencer, &RobotSequencer::onVisionTaskComplete);

    qDebug() << "[SETUP] RobotSequencer <-> RealSenseWidget 연결 완료.";

    startPythonServer();

    ui->widget->setShowPlot(true);
}

MainWindow::~MainWindow()
{
    if (m_pythonProcess && m_pythonProcess->state() != QProcess::NotRunning) {
        qInfo() << "[MAIN] Attempting to close Python server process...";
        m_pythonProcess->terminate();
        if (!m_pythonProcess->waitForFinished(3000)) {
            m_pythonProcess->kill();
        }
        qInfo() << "[MAIN] Python process terminated.";
    }

    emit requestCloseConnection();
    QThread::msleep(100);

    m_robotControllerThread.quit();
    m_robotControllerThread.wait();
    delete ui;
}

void MainWindow::showEvent(QShowEvent *event)
{
    QMainWindow::showEvent(event);
    QTimer::singleShot(100, ui->widget, &RealSenseWidget::startCameraStream);
}

void MainWindow::on_RobotInit_clicked()
{
    switch (m_robotConnectionState)
    {
    case RobotConnectionState::Disconnected:
    {
        updateUiForState(RobotConnectionState::Connecting);
        s_robotStateLabel->setText("Robot Status: CONNECTING...");
        emit requestInitializeRobot();
    }
    break;
    case RobotConnectionState::Connected:
    {
        qDebug() << "[ROBOT] 'Servo ON' button clicked.";
        emit requestServoOn();
    }
    break;

    case RobotConnectionState::ServoOn:
        qInfo() << "[ROBOT] 'Close' button clicked. Disconnecting from robot.";
        emit requestCloseConnection();
        break;

    case RobotConnectionState::Connecting:
        break;
    }
}

void MainWindow::onRobotInitFailed(QString error)
{
    qWarning() << "[ROBOT] Init failed from thread:" << error;
    s_robotStateLabel->setText("Robot Status: FAILED TO CONNECT");
    updateUiForState(RobotConnectionState::Disconnected);
}


MainWindow::RobotConnectionState MainWindow::getConnectionState() const
{
    return m_robotConnectionState;
}

void MainWindow::updateUiForState(RobotConnectionState state)
{
    m_robotConnectionState = state;

    QPushButton* autoButton = ui->AutoMoveButton;

    switch (state) {
    case RobotConnectionState::Disconnected:
        ui->RobotInit->setText("Init");
        ui->RobotInit->setEnabled(true);
        if(autoButton) autoButton->setEnabled(false);
        break;
    case RobotConnectionState::Connecting:
        ui->RobotInit->setText("Connecting...");
        ui->RobotInit->setEnabled(false);
        if(autoButton) autoButton->setEnabled(false);
        break;
    case RobotConnectionState::Connected:
        ui->RobotInit->setText("Servo ON");
        ui->RobotInit->setEnabled(true);
        if(autoButton) autoButton->setEnabled(false);
        break;
    case RobotConnectionState::ServoOn:
        ui->RobotInit->setText("Close");
        ui->RobotInit->setEnabled(true);
        if(autoButton) autoButton->setEnabled(true);
        break;
    }
}

void MainWindow::on_GripperOpenButton_clicked() { emit requestGripperAction(0); }
void MainWindow::on_GripperCloseButton_clicked() { emit requestGripperAction(1); }
void MainWindow::on_ResetPosButton_clicked() { emit requestResetPosition(); }

void MainWindow::on_MoveButton_clicked()
{
    qDebug() << "[MAIN] 'Move' button clicked. Initiating full automated sequence.";
    ui->widget->runFullAutomatedSequence();
}

// ✨ [수정] 'Calc Body Grip' 버튼 클릭 -> onShowXYPlot 호출 (바디 파지 계산 및 시각화)
void MainWindow::on_CalcBodyGripButton_clicked()
{
    qDebug() << "[MAIN] 'Calc Body Grip' button clicked. Requesting Body Grasp calculation/plot.";
    ui->widget->onShowXYPlot();
}

void MainWindow::on_MoveViewButton_clicked()
{
    qDebug() << "[MAIN] 'Move View' button clicked. Requesting handle view pose CALCULATION.";
    ui->widget->onCalculateHandleViewPose();
}

void MainWindow::on_MovepointButton_clicked()
{
    qDebug() << "[MAIN] 'MovepointButton' clicked. Requesting robot MOVE to handle view pose.";
    ui->widget->onMoveToCalculatedHandleViewPose();
}

void MainWindow::on_HandleGrapsButton_clicked()
{
    qDebug() << "[MAIN] 'Grasp Handle' button clicked. Requesting move to *ICP* grasp pose.";
    ui->widget->onMoveToIcpGraspPoseRequested();
}

void MainWindow::on_AlignHangButton_clicked()
{
    qDebug() << "[MAIN] 'Align Hang' (Auto-Slot) clicked. (Note: Should be handled by manual connect)";
}


void MainWindow::updateRobotStateLabel(int state)
{
    ROBOT_STATE eState = (ROBOT_STATE)state;

    if (eState == STATE_STANDBY && g_bServoOnAttempted && getConnectionState() == RobotConnectionState::Connected) {
        updateUiForState(RobotConnectionState::ServoOn);
    }

    if (eState != STATE_STANDBY && eState != STATE_MOVING && getConnectionState() == RobotConnectionState::ServoOn) {
        if (eState == STATE_SAFE_OFF || eState == STATE_SAFE_OFF2 || eState == STATE_EMERGENCY_STOP) {
            updateUiForState(RobotConnectionState::Connected);
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

void MainWindow::updateRobotPoseLabel(const float* pose)
{
    if (ui->RobotPos && pose) {
        QString pose_str = QString("Pose: X:%1 Y:%2 Z:%3 | A:%4 B:%5 C:%6")
        .arg(pose[0], 0, 'f', 1).arg(pose[1], 0, 'f', 1).arg(pose[2], 0, 'f', 1)
            .arg(pose[3], 0, 'f', 1).arg(pose[4], 0, 'f', 1).arg(pose[5], 0, 'f', 1);
        ui->RobotPos->setText(pose_str);
    }
}


void MainWindow::on_AutoMoveButton_clicked()
{
    qDebug() << "[MAIN] 'AutoMoveButton' clicked. Starting full sequence.";
    ui->AutoMoveButton->setEnabled(false);
    emit requestFullAutomation();
}

void MainWindow::onAutomationFinished()
{
    qDebug() << "[MAIN] Full automation sequence finished.";
    ui->AutoMoveButton->setEnabled(true);
}


void MainWindow::startPythonServer()
{
    QString CONDA_ROOT_PATH = "/home/test/anaconda3";
    QString CONDA_ENV_NAME = "dt2";
    QString SCRIPT_PATH = "/home/test/Desktop/DL/detectron2/cup_detector_server.py";

    QString command = QString(
                          "source %1/etc/profile.d/conda.sh && conda activate %2 && python3 %3"
                          ).arg(CONDA_ROOT_PATH).arg(CONDA_ENV_NAME).arg(SCRIPT_PATH);

    qInfo() << "[MAIN] Launching Python Server via bash -c (Conda method)...";
    m_pythonProcess->start("/bin/bash", QStringList() << "-c" << command);

    if (!m_pythonProcess->waitForStarted(5000)) {
        qCritical() << "[MAIN] Failed to start Python server process within 5 seconds.";
        qCritical() << "[MAIN] Error:" << m_pythonProcess->errorString();
    } else {
        qInfo() << "[MAIN] Python server process started successfully (PID:" << m_pythonProcess->processId() << ")";
    }
}

void MainWindow::onPythonServerFinished(int exitCode, QProcess::ExitStatus exitStatus)
{
    if (exitStatus == QProcess::NormalExit) {
        qInfo() << "[PYTHON] Server finished normally with exit code:" << exitCode;
    } else {
        qWarning() << "[PYTHON] Server crashed or finished unexpectedly. Exit code:" << exitCode;
    }
}
