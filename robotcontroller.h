#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#include <QObject>
#include <QVector3D>
#include <QTimer>
#include <QQuaternion>
#include <QMatrix3x3>
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
    void robotStateChanged(int state);
    void robotPoseUpdated(const float* poseMatrix);
    void robotTransformUpdated(const QMatrix4x4 &transform);

public slots:
    void startMonitoring();
    void onMoveRobot(const QVector3D& position_mm, const QVector3D& orientation_deg);
    void onRobotPickAndReturn(const QVector3D& target_pos_mm, const QVector3D& target_ori_deg, const QVector3D& approach_pos_mm, const QVector3D& approach_ori_deg);
    void onResetPosition();
    void onGripperAction(int action);
    void onLiftRotatePlaceSequence(const QVector3D& lift_pos_mm, const QVector3D& lift_ori_deg,
                                   const QVector3D& rotate_pos_mm, const QVector3D& rotate_ori_deg,
                                   const QVector3D& place_pos_mm, const QVector3D& place_ori_deg);

    void onFullPickAndPlaceSequence(
        const QVector3D& pre_grasp_pos_mm, const QVector3D& pre_grasp_ori_deg,
        const QVector3D& grasp_pos_mm, const QVector3D& grasp_ori_deg,
        const QVector3D& lift_pos_mm, const QVector3D& lift_ori_deg,
        const QVector3D& rotate_pos_mm, const QVector3D& rotate_ori_deg,
        const QVector3D& place_pos_mm, const QVector3D& place_ori_deg
        );

    // ✨ [추가] 1. (5cm 뒤) 접근 -> 2. (목표) 파지 2단계 이동을 처리할 슬롯
    void onApproachThenGrasp(const QVector3D& approach_pos_mm, const QVector3D& final_pos_mm, const QVector3D& orientation_deg);

private slots:
    void checkRobotState();

private:
    QTimer *m_timer;
    bool m_angleDebugPrinted;

    void moveToPositionAndWait(const QVector3D& pos_mm, const QVector3D& ori_deg);
    QVector3D rotationMatrixToEulerAngles(const QMatrix3x3& R, const QString& order);

};

#endif // ROBOTCONTROLLER_H
