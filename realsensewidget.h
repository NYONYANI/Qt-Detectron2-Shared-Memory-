#ifndef REALSENSEWIDGET_H
#define REALSENSEWIDGET_H

#include <QWidget>
#include <QImage>
#include <QTimer>
#include <QPainter>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonDocument>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QHBoxLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>
#include <QMatrix4x4>
#include <QtMath>
#include <QList>
#include <QVector3D>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <sys/mman.h>
#include <semaphore.h>
#include "xyplotwidget.h"
#include <GL/glu.h>
#include "DBSCAN.h"

struct GraspingTarget
{
    QVector3D point;
    QVector3D direction;
    QPointF circleCenter;
    QPointF handleCentroid;
};

class PointCloudWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:
    explicit PointCloudWidget(QWidget *parent = nullptr);
    ~PointCloudWidget();

    void updatePointCloud(const rs2::points& points, const rs2::video_frame& color, const QImage& maskOverlay);
    void setTransforms(const QMatrix4x4& baseToTcp, const QMatrix4x4& tcpToCam);
    void updateGraspingPoints(const QVector<QVector3D>& points);
    void updateTargetPoses(const QMatrix4x4& pose, bool show, const QMatrix4x4& pose_y_aligned, bool show_y_aligned);

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

    std::vector<float> m_vertexData;
    rs2::points m_points;
    rs2::video_frame m_colorFrame;
    QImage m_maskOverlay;
    bool m_showOnlyMaskedPoints = false;
    bool m_isZFiltered = false;
    bool m_isFloorFiltered = false;
    std::vector<bool> m_floorPoints;

    QVector<QVector3D> m_graspingPoints;

    QMatrix4x4 m_targetTcpTransform;
    bool m_showTargetPose = false;
    QMatrix4x4 m_targetTcpTransform_Y_Aligned;
    bool m_showTargetPose_Y_Aligned = false;

    friend class RealSenseWidget;

    QMatrix4x4 m_baseToTcpTransform;
    QMatrix4x4 m_tcpToCameraTransform;

    float m_yaw, m_pitch, m_distance, m_panX, m_panY;
    QPoint m_lastPos;
};


class RealSenseWidget : public QWidget
{
    Q_OBJECT

public:
    explicit RealSenseWidget(QWidget *parent = nullptr);
    ~RealSenseWidget();
    void setShowPlot(bool show);

public slots:
    void startCameraStream();
    void captureAndProcess();
    void onRobotTransformUpdated(const QMatrix4x4 &transform);
    void onMoveToYAlignedPoseRequested();

    // (MainWindow에서 호출하기 위해 public으로 이동)
    void onDenoisingToggled();
    void onZFilterToggled();
    void onShowXYPlot();
    void onCalculateTargetPose();
    void onMoveRobotToPreGraspPose();
    void onPickAndReturnRequested();
    void onToggleMaskedPoints();

    // (Move 버튼이 호출할 메인 함수)
    void runFullAutomatedSequence();

signals:
    void requestRobotMove(const QVector3D& position_mm, const QVector3D& orientation_deg);
    void requestGripperAction(int action);
    void requestRobotPickAndReturn(const QVector3D& target_pos_mm, const QVector3D& target_ori_deg, const QVector3D& approach_pos_mm, const QVector3D& approach_ori_deg);
    void requestLiftRotatePlaceSequence(const QVector3D& lift_pos_mm, const QVector3D& lift_ori_deg,
                                        const QVector3D& rotate_pos_mm, const QVector3D& rotate_ori_deg,
                                        const QVector3D& place_pos_mm, const QVector3D& place_ori_deg);

    // (로봇 컨트롤러에게 보낼 전체 시퀀스 시그널)
    void requestFullPickAndPlaceSequence(
        const QVector3D& pre_grasp_pos_mm, const QVector3D& pre_grasp_ori_deg,
        const QVector3D& grasp_pos_mm, const QVector3D& grasp_ori_deg,
        const QVector3D& lift_pos_mm, const QVector3D& lift_ori_deg,
        const QVector3D& rotate_pos_mm, const QVector3D& rotate_ori_deg,
        const QVector3D& place_pos_mm, const QVector3D& place_ori_deg
        );

private slots:
    void updateFrame();
    void checkProcessingResult();

private:
    // (계산 로직을 담당하는 비공개 함수)
    bool calculateGraspingPoses(bool showPlot);

    void findFloorPlaneRANSAC();
    void runDbscanClustering();

    QHBoxLayout *m_layout;
    QLabel *m_colorLabel;
    PointCloudWidget *m_pointCloudWidget;
    QList<XYPlotWidget*> m_plotWidgets;

    QMatrix4x4 m_baseToTcpTransform;
    QMatrix4x4 m_tcpToCameraTransform;

    rs2::pipeline m_pipeline;
    rs2::config m_config;
    rs2::pointcloud m_pointcloud;
    rs2::align m_align;

    rs2::decimation_filter m_dec_filter;
    rs2::spatial_filter m_spat_filter;
    rs2::disparity_transform m_depth_to_disparity;
    rs2::disparity_transform m_disparity_to_depth;
    bool m_isDenoisingOn = false;

    QTimer *m_timer;
    QTimer *m_resultTimer;
    QImage m_currentImage;
    cv::Mat m_latestFrame;

    rs2::points m_capturedPoints;
    QMatrix4x4 m_capturedBaseToTcpTransform;
    std::vector<int> m_uv_to_point_idx;

    int fd_image, fd_result, fd_control;
    void *data_image, *data_result, *data_control;
    sem_t *sem_image, *sem_result, *sem_control;

    QJsonArray m_detectionResults;
    bool m_isProcessing;
    bool m_showPlotWindow;

    QVector<GraspingTarget> m_graspingTargets;

    // ✨ [오타 수정] QMatrix4HMatrix4 -> QMatrix4x4
    QMatrix4x4 m_calculatedTargetPose;
    QVector3D m_calculatedTargetPos_m;
    QVector3D m_calculatedTargetOri_deg;

    QMatrix4x4 m_calculatedTargetPose_Y_Aligned;
    QVector3D m_calculatedTargetOri_deg_Y_Aligned;

    std::vector<int> m_clusterIds;
    const float APPROACH_HEIGHT_M = 0.15f;
    const int IMAGE_WIDTH = 640;
    const int IMAGE_HEIGHT = 480;
    const int IMAGE_CHANNELS = 3;
    const int IMAGE_SIZE = IMAGE_WIDTH * IMAGE_HEIGHT * IMAGE_CHANNELS;
    const int RESULT_SIZE = 100 * 1024;
    const int CONTROL_SIZE = 16;

    enum ControlOffset {
        OFFSET_NEW_FRAME = 0,
        OFFSET_RESULT_READY = 1,
        OFFSET_SHUTDOWN = 2
    };

    QImage cvMatToQImage(const cv::Mat &mat);
    bool initSharedMemory();
    void sendImageToPython(const cv::Mat &mat);
    QJsonArray receiveResultsFromPython();
    void drawMaskOverlay(QImage &image, const QJsonArray &results);
};
#endif // REALSENSEWIDGET_H
