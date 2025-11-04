#ifndef ROBOTSEQUENCER_H
#define ROBOTSEQUENCER_H

#include <QObject>
#include <QVector3D>
#include <QEventLoop> // ✨ [추가]
#include "robotcontroller.h" // RobotController의 public slot을 호출하기 위해 포함

class RobotSequencer : public QObject
{
    Q_OBJECT
public:
    explicit RobotSequencer(QObject *parent = nullptr);

    // MainWindow에서 RobotController 객체를 연결해주기 위한 함수
    void setRobotController(RobotController* controller);

    // ✨ [수정] 자동화 시퀀스 상태 (Step4_5_Filter 제거 및 단계 재정렬)
    enum AutomationState {
        Idle,
        Step1_Capture,
        Step2_Filter, // (Step1_5_Filter 이름 변경)
        Step3_CalcView,
        Step4_MoveView,
        Step5_Capture,
        // Step4_5_Filter, // <-- ✨ [삭제] 두 번째 토글 제거
        Step6_CalcHandle,
        Step7_Reset,
        Step8_Grasp,
        Step9_FinalReset
    };

public slots:
    // RealSenseWidget 또는 MainWindow로부터 시퀀스 실행 요청을 받는 슬롯들
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

    void onApproachThenGrasp(const QVector3D& approach_pos_mm, const QVector3D& final_pos_mm, const QVector3D& orientation_deg);

    // ✨ [추가] 자동화 시퀀스 슬롯
    void onStartFullAutomation();
    void onVisionTaskComplete(); // RealSenseWidget으로부터 완료 신호를 받음
    void onMoveTaskComplete(); // RobotController로부터 이동 완료 신호를 받음


signals:
    // ✨ [추가] RealSenseWidget에 비전 작업을 요청하는 시그널
    void requestVisionCapture();
    void requestVisionMoveViewpoint(); // 뷰포인트 '계산' 요청
    void requestVisionHandlePlot();  // 핸들 플롯 '계산' 요청

    // ✨ [추가] RealSenseWidget에 로봇 이동을 요청하는 시그널 (RSWidget이 포즈를 갖고 있으므로)
    void requestMoveToViewpoint(); // 'MovepointButton' 클릭과 동일한 동작 요청
    void requestGraspHandle();     // 'HandleGrapsButton' 클릭과 동일한 동작 요청

    // ✨ [추가] MainWindow에 시퀀스 완료를 알리는 시그널
    void automationFinished();

    // ✨ [추가] 키 1, 2, 3 필터 요청 시그널
    void requestToggleMask();
    void requestToggleDenoise();
    void requestToggleZFilter();


private:
    RobotController* m_robotController; // 기본 동작을 수행할 RobotController 포인터

    // ✨ [추가] 자동화 시퀀스용 멤버
    AutomationState m_autoState;
    QEventLoop* m_visionWaitLoop;
    QEventLoop* m_moveWaitLoop; // ✨ [추가]
};

#endif // ROBOTSEQUENCER_H
