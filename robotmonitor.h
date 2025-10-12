#ifndef ROBOTMONITOR_H
#define ROBOTMONITOR_H

#include <QObject>
#include <QTimer>
#include <QDebug>
#include <QString>
#include <QMatrix4x4> // QMatrix4x4 사용을 위해 추가
#include "DRFLEx.h"
using namespace DRAFramework;

extern CDRFLEx GlobalDrfl;

class RobotMonitor : public QObject
{
    Q_OBJECT

public:
    explicit RobotMonitor(QObject *parent = nullptr);

signals:
    // 기존 UI 라벨 업데이트를 위한 시그널 (유지)
    void robotStateChanged(int state);
    void robotPoseUpdated(const float* poseMatrix);

    // ✨ [추가] 3D 위젯의 변환 행렬 업데이트를 위한 새로운 시그널
    void robotTransformUpdated(const QMatrix4x4 &transform);


public slots:
    void startMonitoring();

private slots:
    void checkRobotState();

private:
    QTimer *m_timer;
};

#endif // ROBOTMONITOR_H
