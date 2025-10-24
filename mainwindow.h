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
    void requestMoveRobot(const QVector3D& position_mm, const QVector3D& orientation_deg);
    // ✨ [오류 수정] 시그널 이름을 connect 구문과 일치하도록 수정 (Robot 추가)
    void requestRobotPickAndReturn(const QVector3D& target_pos_mm, const QVector3D& target_ori_deg, const QVector3D& approach_pos_mm, const QVector3D& approach_ori_deg);
    void requestResetPosition();
    void requestGripperAction(int action);
    void startRobotMonitoring();

    void requestLiftRotatePlaceSequence(const QVector3D& lift_pos_mm, const QVector3D& lift_ori_deg,
                                        const QVector3D& rotate_pos_mm, const QVector3D& rotate_ori_deg,
                                        const QVector3D& place_pos_mm, const QVector3D& place_ori_deg);
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
    void on_HandlePlotButton_clicked();
    void on_MoveViewButton_clicked();

private:
    Ui::MainWindow *ui;

    QThread m_robotControllerThread;
    RobotController* m_robotController;

    RobotConnectionState m_robotConnectionState;
};
#endif // MAINWINDOW_H
