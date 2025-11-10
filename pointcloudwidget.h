#ifndef POINTCLOUDWIDGET_H
#define POINTCLOUDWIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QImage>
#include <QMatrix4x4>
#include <QVector3D>
#include <QPoint>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>
#include <librealsense2/rs.hpp>
#include <vector>
#include <GL/glu.h>

// RealSenseWidget에 대한 전방 선언 (friend class용)
class RealSenseWidget;

class PointCloudWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT // Q_OBJECT 매크로 유지

public:
    explicit PointCloudWidget(QWidget *parent = nullptr);
    virtual ~PointCloudWidget();

    void updatePointCloud(const rs2::points& points, const rs2::video_frame& color, const QImage& maskOverlay);
    void setTransforms(const QMatrix4x4& baseToTcp, const QMatrix4x4& tcpToCam);
    void updateGraspingPoints(const QVector<QVector3D>& points);
    void updateTargetPoses(const QMatrix4x4& pose, bool show,
                           const QMatrix4x4& pose_y_aligned, bool show_y_aligned,
                           const QMatrix4x4& view_pose, bool show_view_pose);

public slots:
    void updateHandleCenterline(const QVector<QVector3D>& centerline, const QVector<int>& segmentIds);
    void updateRandomGraspPose(const QMatrix4x4& pose, bool show);
    QVector3D setRawBaseFramePoints(const QVector<QVector3D>& points);
    void setRawGraspPose(const QMatrix4x4& pose, bool show);
    void setPCAAxes(const QVector3D& mean, const QVector3D& pc1, const QVector3D& pc2, const QVector3D& normal, bool show);
    void updateDebugLookAtPoint(const QVector3D& point, bool show);
    void updateDebugLine(const QVector3D& p1, const QVector3D& p2, bool show);

signals:
    void denoisingToggled();
    void zFilterToggled();
    void showXYPlotRequested();
    void calculateTargetPoseRequested();
    void moveRobotToPreGraspPoseRequested();
    void pickAndReturnRequested();

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;

private:
    void processPoints(const std::vector<int>& clusterIds = {});
    void drawAxes(float length, float lineWidth = 2.0f);
    void drawGrid(float size, int divisions);
    void drawGraspingSpheres();
    void drawGripper();
    void drawTargetPose();
    void drawTargetPose_Y_Aligned();
    void drawViewPose();
    void drawHandleCenterline();
    void drawRandomGraspPose();

    void drawRawCentroid();
    void drawRawGraspPoint();
    void drawRawGraspPoseAxis();
    void drawPole();
    void drawPCAAxes();
    void drawDebugLookAtPoint();
    void drawDebugLine();
    std::vector<float> m_vertexData;
    rs2::points m_points;
    rs2::video_frame m_colorFrame;
    QImage m_maskOverlay;
    bool m_showOnlyMaskedPoints = false;
    bool m_isZFiltered = false;
    bool m_isFloorFiltered = false;
    std::vector<bool> m_floorPoints;

    QVector<QVector3D> m_graspingPoints;

    QMatrix4x4 m_targetTcpTransform; bool m_showTargetPose = false;
    QMatrix4x4 m_targetTcpTransform_Y_Aligned; bool m_showTargetPose_Y_Aligned = false;
    QMatrix4x4 m_viewPoseTransform; bool m_showViewPose = false;

    QVector<QVector3D> m_handleCenterlinePoints;
    QVector<int> m_handleCenterlineSegmentIds;

    QMatrix4x4 m_randomGraspPose;
    bool m_showRandomGraspPose = false;

    QVector<float> m_rawBaseFramePoints;
    bool m_isRawVizMode = false;
    QVector3D m_rawCentroid;
    bool m_showRawCentroid = false;
    QVector3D m_rawGraspPoint;
    bool m_showRawGraspPoint = false;
    QMatrix4x4 m_rawGraspPose;
    bool m_showRawGraspPose = false;

    QVector3D m_pcaMeanViz;
    QVector3D m_pcaPC1Viz;
    QVector3D m_pcaPC2Viz;
    QVector3D m_pcaNormalViz;
    QVector3D m_debugLookAtPoint;
    bool m_showPCAAxes = false;
    bool m_showDebugLookAtPoint;
    QVector3D m_debugLineP1;
    QVector3D m_debugLineP2;
    bool m_showDebugLine;

    // ✨ friend class 선언
    friend class RealSenseWidget;

    QMatrix4x4 m_baseToTcpTransform;
    QMatrix4x4 m_tcpToCameraTransform;

    float m_yaw, m_pitch, m_distance, m_panX, m_panY, m_panZ;
    QPoint m_lastPos;
};

#endif // POINTCLOUDWIDGET_H
