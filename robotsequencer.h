#ifndef ROBOTSEQUENCER_H
#define ROBOTSEQUENCER_H

#include <QObject>
#include <QVector3D>
#include <QEventLoop>
#include "robotcontroller.h"

class RobotSequencer : public QObject
{
    Q_OBJECT
public:
    explicit RobotSequencer(QObject *parent = nullptr);

    void setRobotController(RobotController* controller);

    enum AutomationState {
        Idle,
        Step1_Capture,
        Step2_Filter,
        Step3_CalcView,
        Step4_MoveView,
        Step5_Capture,
        Step6_CalcHandle,
        Step7_Reset,
        Step8_Grasp,
        Step9_FinalReset
    };

public slots:
    void onRobotPickAndReturn(const QVector3D& target_pos_mm, const QVector3D& target_ori_deg, const QVector3D& approach_pos_mm, const QVector3D& approach_ori_deg);
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

    // ✨ [수정] hang_pose_matrix 인자 제거
    void onApproachThenGrasp(const QVector3D& approach_pos_mm, const QVector3D& final_pos_mm, const QVector3D& orientation_deg);

    void onStartFullAutomation();
    void onVisionTaskComplete();
    void onMoveTaskComplete();

    // ✨ [삭제] onAlignHangSequence 함수 제거

signals:
    void requestVisionCapture();
    void requestVisionMoveViewpoint();
    void requestVisionHandlePlot();

    void requestMoveToViewpoint();
    void requestGraspHandle();

    void automationFinished();

    void requestToggleMask();
    void requestToggleDenoise();
    void requestToggleZFilter();

private:
    RobotController* m_robotController;
    AutomationState m_autoState;
    QEventLoop* m_visionWaitLoop;
    QEventLoop* m_moveWaitLoop;
};

#endif // ROBOTSEQUENCER_H
