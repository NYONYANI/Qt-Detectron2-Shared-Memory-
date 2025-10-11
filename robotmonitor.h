#ifndef ROBOTMONITOR_H
#define ROBOTMONITOR_H

#include <QObject>
#include <QTimer>
#include <QDebug>
#include <QString>
#include "DRFLEx.h"
using namespace DRAFramework;

extern CDRFLEx GlobalDrfl;

class RobotMonitor : public QObject
{
    Q_OBJECT

public:
    explicit RobotMonitor(QObject *parent = nullptr);

signals:
    void robotStateChanged(int state);
    // ✨ [수정] 4x4 변환 행렬(float[16])을 전달하도록 시그널 수정
    void robotPoseUpdated(const float* poseMatrix);

public slots:
    void startMonitoring();

private slots:
    void checkRobotState();

private:
    QTimer *m_timer;
};

#endif // ROBOTMONITOR_H
