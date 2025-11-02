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
    // --- 기본 기능 ---
    void startMonitoring();
    void onMoveRobot(const QVector3D& position_mm, const QVector3D& orientation_deg);
    void onResetPosition();
    void onGripperAction(int action);

    // --- 시퀀스 헬퍼 (RobotSequencer가 호출할 수 있도록 public slot으로 변경) ---
    bool moveToPositionAndWait(const QVector3D& pos_mm, const QVector3D& ori_deg);

    // --- 시퀀스 슬롯 (RobotSequencer로 이동) ---
    // void onRobotPickAndReturn(...); // <--- 이동됨
    // void onLiftRotatePlaceSequence(...); // <--- 이동됨
    // void onFullPickAndPlaceSequence(...); // <--- 이동됨
    // void onApproachThenGrasp(...); // <--- 이동됨

private slots:
    void checkRobotState();

private:
    QTimer *m_timer;
    bool m_angleDebugPrinted;

    // --- 헬퍼 함수 ---
    QVector3D rotationMatrixToEulerAngles(const QMatrix3x3& R, const QString& order);

};

#endif // ROBOTCONTROLLER_H
