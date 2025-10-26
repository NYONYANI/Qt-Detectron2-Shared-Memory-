#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#include <QObject>
#include <QVector3D>
#include <QTimer>
#include <QQuaternion> // ✨ [추가]
#include <QMatrix3x3>  // ✨ [추가] QMatrix4x4 대신 3x3 사용 및 계산 용이성 위해
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

    // ✨ [추가] RealSenseWidget에서 모든 좌표를 받아 전체 시퀀스를 실행하는 슬롯
    void onFullPickAndPlaceSequence(
        const QVector3D& pre_grasp_pos_mm, const QVector3D& pre_grasp_ori_deg,
        const QVector3D& grasp_pos_mm, const QVector3D& grasp_ori_deg,
        const QVector3D& lift_pos_mm, const QVector3D& lift_ori_deg,
        const QVector3D& rotate_pos_mm, const QVector3D& rotate_ori_deg,
        const QVector3D& place_pos_mm, const QVector3D& place_ori_deg
        );

private slots:
    void checkRobotState();

private:
    QTimer *m_timer;
    // ✨ [추가] 각도 디버그 메시지 출력 여부 플래그
    bool m_angleDebugPrinted;

    void moveToPositionAndWait(const QVector3D& pos_mm, const QVector3D& ori_deg);

    // ✨ [추가] Euler 각도 변환 함수 선언 (다양한 규약 지원 위해)
    QVector3D rotationMatrixToEulerAngles(const QMatrix3x3& R, const QString& order);

};

#endif // ROBOTCONTROLLER_H
