#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>
#include <QLabel>
#include <QThread>
#include <QMatrix4x4> // ✨ [추가] QMatrix4x4 헤더 포함
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

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_RobotInit_clicked();
    void updateRobotStateLabel(int state);
    void updateRobotPoseLabel(const float* pose);
    // ✨ [수정] onMoveRobot 슬롯의 선언을 cpp 파일과 일치시킵니다.
    void onMoveRobot(const QMatrix4x4& poseMatrix);

public:
    static QLabel* s_robotStateLabel;

private:
    Ui::MainWindow *ui;
    QThread *m_monitorThread;
    RobotMonitor *m_robotMonitor;
};
#endif // MAINWINDOW_H
