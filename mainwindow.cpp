#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "DRFLEx.h"
#include <QDebug>
#include <QLabel>
#include <QThread>

// ----------------------------------------------------
// [전역 객체, 플래그 및 정적 포인터 정의]
// ----------------------------------------------------

using namespace DRAFramework;

// 💡 GlobalDrfl 객체 정의
CDRFLEx GlobalDrfl;

// ✅ 제어권 상태 추적을 위한 전역 플래그
bool g_bHasControlAuthority = false;
bool g_bServoOnAttempted = false;
bool g_TpInitailizingComplted = false;

// ✅ 전역 콜백 함수에서 UI 라벨에 접근하기 위한 정적 멤버
QLabel* MainWindow::s_robotStateLabel = nullptr;

// ----------------------------------------------------
// [함수 전방 선언]
// ----------------------------------------------------
void OnMonitoringStateCB(const ROBOT_STATE eState);

// ----------------------------------------------------
// [전역 콜백 함수 정의 (mainwindow_old.cpp 방식 적용)]
// ----------------------------------------------------

// 1. 초기화 완료 콜백
void OnTpInitializingCompleted() {
    qDebug() << "TP Initializing Completed.";
    g_TpInitailizingComplted = true;
    GlobalDrfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
}

// 2. 접속 끊김 콜백
void OnDisConnected() {
    qDebug() << "Robot Disconnected. Resetting flags.";
    g_bServoOnAttempted = false;
    g_bHasControlAuthority = false;
    g_TpInitailizingComplted = false;

    if (MainWindow::s_robotStateLabel != nullptr) {
        MainWindow::s_robotStateLabel->setText("Robot Status: DISCONNECTED");
        MainWindow::s_robotStateLabel->adjustSize();
    }
}

// 3. 제어권 콜백
void OnMonitroingAccessControlCB(const MONITORING_ACCESS_CONTROL eTrasnsitControl) {
    switch (eTrasnsitControl) {
        case MONITORING_ACCESS_CONTROL_REQUEST:
            // 다른 제어기가 제어권 요청 시 거부
            GlobalDrfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_RESPONSE_NO);
            qDebug() << "Control Request from others - DENIED";
            break;

        case MONITORING_ACCESS_CONTROL_GRANT:
        {
            g_bHasControlAuthority = true;
            qDebug() << "✅ Control Authority GRANTED!";

            // 제어권 획득 즉시 현재 상태 확인 및 처리
            ROBOT_STATE currentState = GlobalDrfl.GetRobotState();
            qDebug() << "Current State after GRANT:" << (int)currentState;
            OnMonitoringStateCB(currentState);
            break;
        }

        case MONITORING_ACCESS_CONTROL_DENY:
        case MONITORING_ACCESS_CONTROL_LOSS:
        {
            g_bHasControlAuthority = false;
            g_bServoOnAttempted = false;
            qDebug() << "❌ Control Authority LOST/DENIED!";

            // 재요청 시도
            if (g_TpInitailizingComplted) {
                GlobalDrfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
                qDebug() << "Re-requesting control authority...";
            }
            break;
        }

        default:
            break;
    }
}

// 4. 로봇 상태 콜백
void OnMonitoringStateCB(const ROBOT_STATE eState) {
    qDebug() << "Robot State Callback:" << (int)eState
             << "| Has Control:" << g_bHasControlAuthority;

    if (!g_bHasControlAuthority) {
        qDebug() << "No control authority - skipping state handling";
        return;
    }

    switch (eState) {
        case STATE_SAFE_STOP:
            qDebug() << "Handling SAFE_STOP...";
            GlobalDrfl.SetSafeStopResetType(SAFE_STOP_RESET_TYPE_DEFAULT);
            GlobalDrfl.SetRobotControl(CONTROL_RESET_SAFET_STOP);
            break;

        case STATE_SAFE_STOP2:
            qDebug() << "Handling SAFE_STOP2...";
            GlobalDrfl.SetRobotControl(CONTROL_RECOVERY_SAFE_STOP);
            break;

        case STATE_SAFE_OFF2:
            qDebug() << "Handling SAFE_OFF2...";
            GlobalDrfl.SetRobotControl(CONTROL_RECOVERY_SAFE_OFF);
            break;

        case STATE_SAFE_OFF:
            qDebug() << "🔧 SAFE_OFF detected - Sending SERVO_ON command...";
            if (GlobalDrfl.SetRobotControl(CONTROL_SERVO_ON)) {
                qDebug() << "✅ SERVO_ON command sent successfully";
            } else {
                qDebug() << "❌ SERVO_ON command failed";
            }
            break;

        case STATE_STANDBY:
            if (!g_bServoOnAttempted) {
                qDebug() << "STANDBY state - Ensuring servo is ON...";
                if (GlobalDrfl.SetRobotControl(CONTROL_SERVO_ON)) {
                    g_bServoOnAttempted = true;
                    qDebug() << "✅ Servo ON confirmed in STANDBY";
                }
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

    // 정적 멤버에 라벨 포인터 설정
    s_robotStateLabel = ui->RobotState;
    if (s_robotStateLabel != nullptr) {
        s_robotStateLabel->setText("Robot Status: Disconnected");
        s_robotStateLabel->adjustSize();
    }

    // Capture 버튼과 RealSenseWidget의 슬롯 연결
    connect(ui->CaptureButton, &QPushButton::clicked,
            ui->widget, &RealSenseWidget::captureAndProcess);

    // 모니터링 스레드 및 객체 생성
    m_monitorThread = new QThread(this);
    m_robotMonitor = new RobotMonitor();
    m_robotMonitor->moveToThread(m_monitorThread);

    // 시그널-슬롯 연결
    connect(m_monitorThread, &QThread::started, m_robotMonitor, &RobotMonitor::startMonitoring);
    connect(m_robotMonitor, &RobotMonitor::robotStateChanged, this, &MainWindow::updateRobotStateLabel);
    connect(m_monitorThread, &QThread::finished, m_robotMonitor, &QObject::deleteLater);

    qDebug() << "MainWindow initialized";
}

MainWindow::~MainWindow()
{
    if (m_monitorThread && m_monitorThread->isRunning()) {
        m_monitorThread->quit();
        m_monitorThread->wait(1000); // 1초간 대기
    }
    GlobalDrfl.CloseConnection(); // 로봇 연결 종료
    delete ui;
}

void MainWindow::on_RobotInit_clicked()
{
    qDebug() << "========================================";
    qDebug() << "로봇 초기화 시작!";
    qDebug() << "========================================";

    const char* robot_ip = "192.168.137.100";

    // 플래그 초기화
    g_bServoOnAttempted = false;
    g_bHasControlAuthority = false;
    g_TpInitailizingComplted = false;

    // 1. 콜백 함수 등록
    GlobalDrfl.set_on_tp_initializing_completed(OnTpInitializingCompleted);
    GlobalDrfl.set_on_disconnected(OnDisConnected);
    GlobalDrfl.set_on_monitoring_access_control(OnMonitroingAccessControlCB);
    GlobalDrfl.set_on_monitoring_state(OnMonitoringStateCB);

    // 2. 로봇 연결
    bool bSuccess = GlobalDrfl.open_connection(robot_ip);

    if (bSuccess)
    {
        qDebug() << "✅ 로봇 연결 성공!";
        if (s_robotStateLabel) {
            s_robotStateLabel->setText("Robot Status: CONNECTING...");
            s_robotStateLabel->adjustSize();
        }

        // 3. 시스템 정보 확인 및 설정
        SYSTEM_VERSION tSysVerion{};
        GlobalDrfl.get_system_version(&tSysVerion);
        GlobalDrfl.setup_monitoring_version(1);
        qDebug() << "System version:" << tSysVerion._szController;
        qDebug() << "Library version:" << GlobalDrfl.get_library_version();

        qDebug() << "Setting robot mode and system...";
        if (!GlobalDrfl.set_robot_mode(ROBOT_MODE_AUTONOMOUS)) {
             qDebug() << "❌ Failed to set robot mode";
        }
        if (!GlobalDrfl.set_robot_system(ROBOT_SYSTEM_REAL)) {
            qDebug() << "❌ Failed to set robot system";
        }

        // 4. 제어권 강제 요청 (초기화 콜백에서 수행하지만, 선제적으로도 요청)
        GlobalDrfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
        qDebug() << "Control authority requested.";

        // 5. 모니터링 스레드 시작
        if (!m_monitorThread->isRunning()) {
            m_monitorThread->start();
            qDebug() << "✅ Robot monitoring thread started.";
        }
    }
    else
    {
        qWarning() << "❌ 로봇 연결 실패! IP 및 네트워크 확인 필요.";
        if (s_robotStateLabel) {
            s_robotStateLabel->setText("Robot Status: FAILED TO CONNECT");
            s_robotStateLabel->adjustSize();
        }
    }
}

void MainWindow::updateRobotStateLabel(int state)
{
    ROBOT_STATE eState = (ROBOT_STATE)state;

    if (s_robotStateLabel != nullptr)
    {
        QString stateText;
        QString controlStatus = g_bHasControlAuthority ? " [제어권 O]" : " [제어권 X]";

        switch (eState) {
            case STATE_NOT_READY:
                stateText = "Not Ready (초기화 필요)";
                break;
            case STATE_INITIALIZING:
                stateText = "Initializing (초기화 중)";
                break;
            case STATE_STANDBY:
                stateText = "✅ Standby (대기 중)";
                break;
            case STATE_MOVING:
                stateText = "Moving (동작 중)";
                break;
            case STATE_EMERGENCY_STOP:
                stateText = "🚨 Emergency Stop (비상 정지)";
                break;
            case STATE_SAFE_STOP:
                stateText = "⚠️ Safe Stop (안전 정지)";
                break;
            case STATE_SAFE_OFF:
                stateText = "⚠️ Safe Off (서보 꺼짐)";
                break;
            case STATE_SAFE_STOP2:
                stateText = "⚠️ Safe Stop 2 (안전 정지 2)";
                break;
            case STATE_SAFE_OFF2:
                stateText = "Safe Off 2 (서보 꺼짐 2)";
                break;
            case STATE_RECOVERY:
                stateText = "Recovery (복구 중)";
                break;
            case STATE_TEACHING:
                stateText = "Teaching (티칭 모드)";
                break;
            default:
                stateText = QString("Unknown State (%1)").arg((int)eState);
                break;
        }

        s_robotStateLabel->setText("Robot Status: " + stateText + controlStatus);
        s_robotStateLabel->adjustSize();
    }
}
