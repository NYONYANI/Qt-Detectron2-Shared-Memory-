#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>
#include <QLabel>
#include <QThread> // QThread 사용을 위해 추가
#include "DRFL.h"
#include "DRFLEx.h"
#include "robotmonitor.h" // RobotMonitor 클래스 사용을 위해 추가
using namespace DRAFramework;

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
    // 모니터 스레드로부터 상태 업데이트 시그널을 받을 슬롯
    void updateRobotStateLabel(int state);

public: // 👈 전역 콜백 접근을 위해 Public으로 유지
    static QLabel* s_robotStateLabel;

private:
    Ui::MainWindow *ui;

    // 💡 누락된 멤버 변수 선언 추가
    QThread *m_monitorThread;
    RobotMonitor *m_robotMonitor;
};
#endif // MAINWINDOW_H
