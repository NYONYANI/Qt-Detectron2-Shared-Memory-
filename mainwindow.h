#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>
#include <QLabel>
#include <QThread>
#include "DRFL.h"
#include "DRFLEx.h"
#include "robotmonitor.h"
#include "realsensewidget.h" // RealSenseWidget 헤더 포함

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

public:
    // 다른 파일에서 UI 라벨에 접근할 수 있도록 static으로 선언
    static QLabel* s_robotStateLabel;

private:
    Ui::MainWindow *ui;
    QThread *m_monitorThread;
    RobotMonitor *m_robotMonitor;
};
#endif // MAINWINDOW_H
