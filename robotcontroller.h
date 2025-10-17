#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#include <QObject>
#include <QVector3D>
#include <QTimer>
#include "DRFLEx.h"

using namespace DRAFramework;
extern CDRFLEx GlobalDrfl;
extern bool g_bHasControlAuthority;

class RobotController : public QObject
{
    Q_OBJECT
public:
    explicit RobotController(QObject *parent = nullptr);

signals:
    // UI 업데이트를 위한 신호 (기존 RobotMonitor의 역할)
    void robotStateChanged(int state);
    void robotPoseUpdated(const float* poseMatrix);
    void robotTransformUpdated(const QMatrix4x4 &transform);

public slots:
    // MainWindow에서 호출될 슬롯들
    void startMonitoring();
    void onMoveRobot(const QVector3D& position_mm, const QVector3D& orientation_deg);
    void onRobotPickAndReturn(const QVector3D& target_pos_mm, const QVector3D& target_ori_deg, const QVector3D& approach_pos_mm, const QVector3D& approach_ori_deg);
    void onResetPosition();
    void onGripperAction(int action);

private slots:
    // 주기적인 상태 확인을 위한 내부 슬롯
    void checkRobotState();

private:
    QTimer *m_timer; // 상태 확인을 위한 타이머
};

#endif // ROBOTCONTROLLER_H
