#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>
#include <QLabel>
#include <QThread>
#include <QMatrix4x4>
#include <QVector3D>
#include "DRFL.h"
#include "DRFLEx.h"
#include "robotmonitor.h"
#include "realsensewidget.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

private slots:
    void on_RobotInit_clicked();
    void updateRobotStateLabel(int state);
    void updateRobotPoseLabel(const float* pose);
    void onMoveRobot(const QVector3D& position_mm, const QVector3D& orientation_deg);
    void on_ResetPosButton_clicked();
    void on_GripperOpenButton_clicked();
    void on_GripperCloseButton_clicked();
    void onGripperAction(int action);
    void onRobotPickAndReturn(const QVector3D& target_pos_mm, const QVector3D& target_ori_deg, const QVector3D& approach_pos_mm, const QVector3D& approach_ori_deg);
    void on_MoveButton_clicked(); // ✨ [추가] MoveButton 슬롯

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    static QLabel* s_robotStateLabel;
    static MainWindow* s_instance;
    bool m_isGripperOpenPending;

private:
    Ui::MainWindow *ui;
    QThread *m_monitorThread;
    RobotMonitor *m_robotMonitor;
    bool m_isWaitingForMoveCompletion;
};
#endif // MAINWINDOW_H
