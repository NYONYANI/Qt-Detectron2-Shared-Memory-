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

class RealSenseWidget;

class PointCloudWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

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

    // ✨ [수정] 원본 PCA 축 설정 (길이 인자 제거, 고정 길이 시각화용)
    void setOriginalPCAAxes(const QVector3D& mean, const QVector3D& pc1, const QVector3D& pc2, const QVector3D& normal);

    void updateDebugLookAtPoint(const QVector3D& point, bool show);
    void updateDebugLine(const QVector3D& p1, const QVector3D& p2, bool show);
    void updateDebugNormal(const QVector3D& p1, const QVector3D& p2, bool show);
    void updateVerticalLine(const QVector3D& p1, const QVector3D& p2, bool show);
    void updateHangCenterPoint(const QVector3D& point, bool show);
    void updateTransformedHandleCloud(const QVector<QVector3D>& points, bool show);
    void updateGraspToBodyLine(const QVector3D& graspPoint, const QVector3D& bodyCenter, bool show);

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
    void drawOriginalPCAAxes(); // ✨ [추가] 원본 PCA 축 그리기 (점선)

    void drawDebugLookAtPoint();
    void drawDebugLine();
    void drawDebugNormal();
    void drawVerticalLine();
    void drawHangCenterPoint();
    void drawTransformedHandleCloud();
    void drawGraspToBodyLine();

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
    bool m_showPCAAxes = false;

    // ✨ [수정] 원본 PCA 데이터
    QVector3D m_pcaMeanOriginal;
    QVector3D m_pcaPC1Original;
    QVector3D m_pcaPC2Original;
    QVector3D m_pcaNormalOriginal;
    bool m_showOriginalPCAAxes = false;

    QVector3D m_debugLookAtPoint;
    bool m_showDebugLookAtPoint;
    QVector3D m_debugLineP1;
    QVector3D m_debugLineP2;
    bool m_showDebugLine;
    QVector3D m_debugNormalP1;
    QVector3D m_debugNormalP2;
    bool m_showDebugNormal;

    QVector3D m_verticalLineP1;
    QVector3D m_verticalLineP2;
    bool m_showVerticalLine;

    QVector3D m_hangCenterViz;
    bool m_showHangCenter;

    QVector<QVector3D> m_transformedHandlePoints;
    bool m_showTransformedHandleCloud;

    QVector3D m_graspToBodyP1;
    QVector3D m_graspToBodyP2;
    bool m_showGraspToBodyLine;

    friend class RealSenseWidget;

    QMatrix4x4 m_baseToTcpTransform;
    QMatrix4x4 m_tcpToCameraTransform;

    float m_yaw, m_pitch, m_distance, m_panX, m_panY, m_panZ;
    QPoint m_lastPos;
};

#endif // POINTCLOUDWIDGET_H
