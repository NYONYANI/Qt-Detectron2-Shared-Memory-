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

//
// --- (콜백 함수들은 변경 없음) ---
//
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
    // ✨ [오류 수정] 실수로 삭제되었던 생성자 초기화 리스트 복원
    , m_robotConnectionState(RobotConnectionState::Disconnected)
{
    ui->setupUi(this);
    s_robotStateLabel = ui->RobotState;
    MainWindow::s_instance = this;
    s_robotStateLabel->setText("Robot Status: Disconnected");

    // --- ✨ [수정] RobotController 및 RobotSequencer 스레드 설정 ---
    m_robotController = new RobotController();
    m_robotController->moveToThread(&m_robotControllerThread);
    connect(&m_robotControllerThread, &QThread::finished, m_robotController, &QObject::deleteLater);

    m_robotSequencer = new RobotSequencer(); // ✨ [추가] Sequencer 생성
    m_robotSequencer->setRobotController(m_robotController); // ✨ [추가] Controller 주입
    m_robotSequencer->moveToThread(&m_robotControllerThread); // ✨ [추가] 동일 스레드로 이동
    connect(m_robotController, &RobotController::moveFinished,
            m_robotSequencer, &RobotSequencer::onMoveTaskComplete, Qt::QueuedConnection);

    // --- MainWindow -> RobotController (기본 명령) ---
    // connect(this, &MainWindow::requestMoveRobot, m_robotController, &RobotController::onMoveRobot); // ✨ [삭제] 아래 RealSenseWidget 연결에서 대체
    connect(this, &MainWindow::requestResetPosition, m_robotController, &RobotController::onResetPosition);
    connect(this, &MainWindow::requestGripperAction, m_robotController, &RobotController::onGripperAction);
    // connect(this, &MainWindow::startRobotMonitoring, m_robotController, &RobotController::startMonitoring); // ✨ [삭제]

    // ✨ [추가] Init, Servo, Close 시그널 연결
    connect(this, &MainWindow::requestInitializeRobot, m_robotController, &RobotController::onInitializeRobot);
    connect(this, &MainWindow::requestServoOn, m_robotController, &RobotController::onServoOn);
    connect(this, &MainWindow::requestCloseConnection, m_robotController, &RobotController::onCloseConnection);


    // --- MainWindow -> RobotSequencer (시퀀스 명령) ---
    // (이 시그널들은 this가 아닌 RealSenseWidget에서 발생하지만, 편의상 this를 거침)
    connect(this, &MainWindow::requestRobotPickAndReturn, m_robotSequencer, &RobotSequencer::onRobotPickAndReturn);
    connect(this, &MainWindow::requestLiftRotatePlaceSequence, m_robotSequencer, &RobotSequencer::onLiftRotatePlaceSequence);

    // ✨ [추가] 자동화 시퀀스 시그널/슬롯 연결
    connect(this, &MainWindow::requestFullAutomation, m_robotSequencer, &RobotSequencer::onStartFullAutomation);
    connect(m_robotSequencer, &RobotSequencer::automationFinished, this, &MainWindow::onAutomationFinished);


    // --- RobotController -> MainWindow (상태 업데이트) ---
    connect(m_robotController, &RobotController::robotStateChanged, this, &MainWindow::updateRobotStateLabel);
    connect(m_robotController, &RobotController::robotPoseUpdated, this, &MainWindow::updateRobotPoseLabel);
    connect(m_robotController, &RobotController::robotTransformUpdated, ui->widget, &RealSenseWidget::onRobotTransformUpdated);

    // ✨ [추가] 초기화 실패 시그널 연결
    connect(m_robotController, &RobotController::initializationFailed, this, &MainWindow::onRobotInitFailed);


    m_robotControllerThread.start();

    // --- UI 위젯 시그널 연결 ---
    // (AutoMoveButton 등 .ui에서 이름이 일치하는 슬롯은 자동 연결됨)

    // ✨ [수정] CaptureButton을 수동으로 연결
    // 람다를 사용해 captureAndProcess(false)를 호출 (isAutoSequence = false)
    connect(ui->CaptureButton, &QPushButton::clicked, ui->widget, [=](){
        ui->widget->captureAndProcess(false);
    });

    // --- RealSenseWidget -> MainWindow (기본 명령 전달용) ---

    // ✨ [수정] RealSenseWidget의 requestRobotMove를 RobotController의 *블로킹* 슬롯에 연결
    connect(ui->widget, &RealSenseWidget::requestRobotMove, m_robotController, &RobotController::moveToPositionAndWait);

    connect(ui->widget, &RealSenseWidget::requestGripperAction, this, &MainWindow::requestGripperAction);

    // --- ✨ [수정] RealSenseWidget -> RobotSequencer (시퀀스 명령 전달용) ---
    connect(ui->widget, &RealSenseWidget::requestRobotPickAndReturn, m_robotSequencer, &RobotSequencer::onRobotPickAndReturn);

    connect(ui->widget, &RealSenseWidget::requestLiftRotatePlaceSequence,
            m_robotSequencer, &RobotSequencer::onLiftRotatePlaceSequence); // ✨ [수정] m_robotSequencer로 연결

    // (이 시그널은 this를 거치지 않고 직접 연결)
    connect(ui->widget, &RealSenseWidget::requestFullPickAndPlaceSequence,
            m_robotSequencer, &RobotSequencer::onFullPickAndPlaceSequence); // ✨ [수정] m_robotSequencer로 연결
    connect(ui->widget, &RealSenseWidget::requestApproachThenGrasp,
            m_robotSequencer, &RobotSequencer::onApproachThenGrasp); // ✨ [수정] m_robotSequencer로 연결


    // --- ✨ [수정] RobotSequencer <-> RealSenseWidget 브릿지 연결 ---
    // (모든 주석 해제 + 필터 3줄 추가)

    // Sequencer -> Vision (작업 요청)

    // ✨ [수정] 람다(lambda)를 사용하여 captureAndProcess(true)를 호출 (컴파일 오류 수정)
    connect(m_robotSequencer, &RobotSequencer::requestVisionCapture, ui->widget, [=](){
        ui->widget->captureAndProcess(true); // 'true'를 전달하여 자동 시퀀스임을 알림
    });

    connect(m_robotSequencer, &RobotSequencer::requestVisionMoveViewpoint, ui->widget, &RealSenseWidget::onCalculateHandleViewPose);
    connect(m_robotSequencer, &RobotSequencer::requestVisionHandlePlot, ui->widget, [=](){
        ui->widget->onShowHandlePlot(false); // 자동 시퀀스 중에는 창을 띄우지 않음 (false)
    });
    connect(m_robotSequencer, &RobotSequencer::requestMoveToViewpoint, ui->widget, &RealSenseWidget::onMoveToCalculatedHandleViewPose);
    connect(m_robotSequencer, &RobotSequencer::requestGraspHandle, ui->widget, &RealSenseWidget::onMoveToRandomGraspPoseRequested);

    // ✨ [추가] Sequencer -> Vision (필터 요청)
    connect(m_robotSequencer, &RobotSequencer::requestToggleMask, ui->widget, &RealSenseWidget::onToggleMaskedPoints);
    connect(m_robotSequencer, &RobotSequencer::requestToggleDenoise, ui->widget, &RealSenseWidget::onDenoisingToggled);
    connect(m_robotSequencer, &RobotSequencer::requestToggleZFilter, ui->widget, &RealSenseWidget::onZFilterToggled);

    // Vision -> Sequencer (완료 신호)
    connect(ui->widget, &RealSenseWidget::visionTaskComplete, m_robotSequencer, &RobotSequencer::onVisionTaskComplete);

    qDebug() << "[SETUP] RobotSequencer <-> RealSenseWidget 연결 완료.";


    ui->widget->setShowPlot(true);
}

MainWindow::~MainWindow()
{
    // ✨ [수정] 스레드 종료 전 Close 요청
    // (이미 연결이 끊겼거나 연결되지 않았다면 CloseConnection은 아무것도 하지 않음)
    emit requestCloseConnection();
    QThread::msleep(100); // DRFL이 닫힐 시간을 잠시 줌

    m_robotControllerThread.quit();
    m_robotControllerThread.wait();
    // GlobalDrfl.CloseConnection(); // ✨ [삭제] 스레드에서 직접 닫도록 변경
    delete ui;
}

//
// --- (showEvent 및 나머지 UI 슬롯, 상태 업데이트 함수들은 변경 없음) ---
//

void MainWindow::showEvent(QShowEvent *event)
{
    QMainWindow::showEvent(event);
    // 윈도우가 완전히 표시된 후 카메라 스트림 시작
    QTimer::singleShot(100, ui->widget, &RealSenseWidget::startCameraStream);
}


//
// ✨ [수정] on_RobotInit_clicked: 실제 로직 대신 시그널 emit
//
void MainWindow::on_RobotInit_clicked()
{
    switch (m_robotConnectionState) // ✨ [오류 수정] 이제 m_robotConnectionState가 선언되어 있음
    {
    case RobotConnectionState::Disconnected:
    {
        updateUiForState(RobotConnectionState::Connecting);
        s_robotStateLabel->setText("Robot Status: CONNECTING...");
        emit requestInitializeRobot(); // ✨ 스레드로 Init 요청
    }
    break;
    case RobotConnectionState::Connected:
    {
        qDebug() << "[ROBOT] 'Servo ON' button clicked.";
        emit requestServoOn(); // ✨ 스레드로 Servo ON 요청
    }
    break;

    case RobotConnectionState::ServoOn:
        qInfo() << "[ROBOT] 'Close' button clicked. Disconnecting from robot.";
        emit requestCloseConnection(); // ✨ 스레드로 Close 요청
        break;

    case RobotConnectionState::Connecting:
        // 연결 중에는 아무것도 하지 않음
        break;
    }
}

//
// ✨ [추가] Init 실패 시 GUI를 업데이트하는 슬롯
//
void MainWindow::onRobotInitFailed(QString error)
{
    qWarning() << "[ROBOT] Init failed from thread:" << error;
    s_robotStateLabel->setText("Robot Status: FAILED TO CONNECT");
    updateUiForState(RobotConnectionState::Disconnected);
}


MainWindow::RobotConnectionState MainWindow::getConnectionState() const
{
    return m_robotConnectionState; // ✨ [오류 수정] 이제 m_robotConnectionState가 선언되어 있음
}

void MainWindow::updateUiForState(RobotConnectionState state)
{
    m_robotConnectionState = state; // ✨ [오류 수정] 이제 m_robotConnectionState가 선언되어 있음

    // ✨ [수정] .ui 파일의 버튼 이름('AutoMoveButton')을 정확히 참조
    QPushButton* autoButton = ui->AutoMoveButton; // b 소문자 (사용자가 .ui에서 바꿨다고 했지만, 업로드된 파일 기준)
    //
    // !!! 만약 .ui 파일을 정말 AutoMoveButton (B 대문자)로 수정했다면,
    // !!! 아래 4줄의 autoButton 참조를 모두 ui->AutoMoveButton 으로 바꿔야 합니다.
    //
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

void MainWindow::on_HandlePlotButton_clicked()
{
    qDebug() << "[MAIN] 'Handle Plot' button clicked. Requesting handle PCA plot.";
    ui->widget->onShowHandlePlot(true);
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
    qDebug() << "[MAIN] 'Grasp Handle' button clicked. Requesting move to random grasp pose.";
    ui->widget->onMoveToRandomGraspPoseRequested();
}


void MainWindow::updateRobotStateLabel(int state)
{
    ROBOT_STATE eState = (ROBOT_STATE)state;

    if (eState == STATE_STANDBY && g_bServoOnAttempted && getConnectionState() == RobotConnectionState::Connected) {
        updateUiForState(RobotConnectionState::ServoOn);
    }

    if (eState != STATE_STANDBY && eState != STATE_MOVING && getConnectionState() == RobotConnectionState::ServoOn) {
        if (eState == STATE_SAFE_OFF || eState == STATE_SAFE_OFF2 || eState == STATE_EMERGENCY_STOP) {
            updateUiForState(RobotConnectionState::Connected); // 서보가 꺼진 상태로 UI 변경
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

// ✨ [수정] 자동화 버튼 슬롯 (.ui 파일에 맞춰 b 소문자로 변경)
void MainWindow::on_AutoMoveButton_clicked()
{
    qDebug() << "[MAIN] 'AutoMoveButton' clicked. Starting full sequence.";

    // 버튼 비활성화 (시퀀스 중복 실행 방지)
    ui->AutoMoveButton->setEnabled(false);

    emit requestFullAutomation();
}

// ✨ [추가] 자동화 완료 슬롯
void MainWindow::onAutomationFinished()
{
    qDebug() << "[MAIN] Full automation sequence finished.";

    // 시퀀스 완료 시 버튼 다시 활성화
    ui->AutoMoveButton->setEnabled(true);
}
