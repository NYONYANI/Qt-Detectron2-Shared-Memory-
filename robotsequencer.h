#ifndef ROBOTSEQUENCER_H
#define ROBOTSEQUENCER_H

#include <QObject>
#include <QVector3D>
#include "robotcontroller.h" // RobotController의 public slot을 호출하기 위해 포함

class RobotSequencer : public QObject
{
    Q_OBJECT
public:
    explicit RobotSequencer(QObject *parent = nullptr);

    // MainWindow에서 RobotController 객체를 연결해주기 위한 함수
    void setRobotController(RobotController* controller);

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


private:
    RobotController* m_robotController; // 기본 동작을 수행할 RobotController 포인터
};

#endif // ROBOTSEQUENCER_H
