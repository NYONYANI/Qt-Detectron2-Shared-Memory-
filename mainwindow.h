#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>
#include <QLabel>
#include <QThread>
#include <QMatrix4x4>
#include <QVector3D>
#include <QShowEvent>
#include "DRFL.h"
#include "DRFLEx.h"
#include "realsensewidget.h"
#include "robotcontroller.h"
#include "robotsequencer.h"
#include <QProcess>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    enum class RobotConnectionState {
        Disconnected,
        Connecting,
        Connected,
        ServoOn
    };

    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    static QLabel* s_robotStateLabel;
    static MainWindow* s_instance;
    bool m_isGripperOpenPending;

    void updateUiForState(RobotConnectionState state);
    RobotConnectionState getConnectionState() const;

signals:
    // --- RobotController로 직접 가는 시그널 ---
    void requestMoveRobot(const QVector3D& position_mm, const QVector3D& orientation_deg);
    void requestResetPosition();
    void requestGripperAction(int action);

    // ✨ Init/Servo/Close 요청 시그널
    void requestInitializeRobot();
    void requestServoOn();
    void requestCloseConnection();


    // --- RobotSequencer로 가는 시그널 ---
    void requestRobotPickAndReturn(const QVector3D& target_pos_mm, const QVector3D& target_ori_deg, const QVector3D& approach_pos_mm, const QVector3D& approach_ori_deg);
    void requestLiftRotatePlaceSequence(const QVector3D& lift_pos_mm, const QVector3D& lift_ori_deg,
                                        const QVector3D& rotate_pos_mm, const QVector3D& rotate_ori_deg,
                                        const QVector3D& place_pos_mm, const QVector3D& place_ori_deg);

    // ✨ 자동화 시퀀스 시작 요청
    void requestFullAutomation();

protected:
    void showEvent(QShowEvent *event) override;

private slots:
    void on_RobotInit_clicked();
    void updateRobotStateLabel(int state);
    void updateRobotPoseLabel(const float* pose);
    void on_ResetPosButton_clicked();
    void on_GripperOpenButton_clicked();
    void on_GripperCloseButton_clicked();
    void on_MoveButton_clicked();

    // ✨ [수정] HandlePlotButton -> CalcBodyGripButton 변경
    void on_CalcBodyGripButton_clicked();

    void on_MoveViewButton_clicked();
    void on_MovepointButton_clicked();
    void on_HandleGrapsButton_clicked();

    // ✨ 로봇 스레드로부터 초기화 실패 시그널을 받는 슬롯
    void onRobotInitFailed(QString error);

    void on_AutoMoveButton_clicked();

    // ✨ 시퀀스 완료 시 UI 업데이트용 슬롯
    void onAutomationFinished();

    // ✨ UI에 새로 추가한 버튼 슬롯
    void on_AlignHangButton_clicked();
    void onPythonServerFinished(int exitCode, QProcess::ExitStatus exitStatus);

private:
    Ui::MainWindow *ui;

    QThread m_robotControllerThread;
    RobotController* m_robotController;
    RobotSequencer* m_robotSequencer;

    RobotConnectionState m_robotConnectionState;
    QProcess* m_pythonProcess;
    void startPythonServer();
};
#endif // MAINWINDOW_H
