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
#include "handleplotwidget.h"
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <QMatrix3x3>
#include "DRFLEx.h"
#include <QElapsedTimer>
#include <QDialog>
#include <QVBoxLayout>


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
    void captureAndProcess(bool isAutoSequence = false);
    void onRobotTransformUpdated(const QMatrix4x4 &transform);
    void onMoveToYAlignedPoseRequested();
    void onDenoisingToggled();
    void onZFilterToggled();
    void onShowXYPlot();
    void onCalculateTargetPose();
    void onMoveRobotToPreGraspPose();
    void onPickAndReturnRequested();
    void onToggleMaskedPoints();

    void onShowHandlePlot(bool showWindow = true);
    void onCalculateHandleViewPose();
    void onMoveToCalculatedHandleViewPose();
    void runFullAutomatedSequence();
    void onMoveToRandomGraspPoseRequested();
    void onMoveToIcpGraspPoseRequested();
    void onShowICPVisualization();


signals:
    void requestRobotMove(const QVector3D& position_mm, const QVector3D& orientation_deg);
    void requestGripperAction(int action);
    void requestRobotPickAndReturn(const QVector3D& target_pos_mm, const QVector3D& target_ori_deg, const QVector3D& approach_pos_mm, const QVector3D& approach_ori_deg);
    void requestLiftRotatePlaceSequence(const QVector3D& lift_pos_mm, const QVector3D& lift_ori_deg,
                                        const QVector3D& rotate_pos_mm, const QVector3D& rotate_ori_deg,
                                        const QVector3D& place_pos_mm, const QVector3D& place_ori_deg);
    void requestFullPickAndPlaceSequence(
        const QVector3D& pre_grasp_pos_mm, const QVector3D& pre_grasp_ori_deg,
        const QVector3D& grasp_pos_mm, const QVector3D& grasp_ori_deg,
        const QVector3D& lift_pos_mm, const QVector3D& lift_ori_deg,
        const QVector3D& rotate_pos_mm, const QVector3D& rotate_ori_deg,
        const QVector3D& place_pos_mm, const QVector3D& place_ori_deg
        );
    void requestHandleCenterlineUpdate(const QVector<QVector3D>& centerline, const QVector<int>& segmentIds);
    void requestRandomGraspPoseUpdate(const QMatrix4x4& pose, bool show);

    void requestApproachThenGrasp(const QVector3D& approach_pos_mm, const QVector3D& final_pos_mm, const QVector3D& orientation_deg);
    void visionTaskComplete();

    void requestRawGraspPoseUpdate(const QMatrix4x4& pose, bool show);

private slots:
    void updateFrame();
    void checkProcessingResult();

private:
    struct HandleAnalysisResult {
        int cupIndex = -1;
        QVector<QVector3D> handlePoints3D;
        QVector<QPointF> projectedPoints2D;
        Eigen::Vector3f pcaMean;
        Eigen::Vector3f pcaPC1;
        Eigen::Vector3f pcaPC2;
        Eigen::Vector3f pcaNormal;
        bool isValid = false;
        float distanceToRobot = std::numeric_limits<float>::max();
    };


    bool calculateGraspingPoses(bool showPlot);
    bool calculatePCA(const QVector<QVector3D>& points,
                      QVector<QPointF>& projectedPoints,
                      Eigen::Vector3f& outMean,
                      Eigen::Vector3f& outPC1,
                      Eigen::Vector3f& outPC2,
                      Eigen::Vector3f& outNormal);
    QVector3D extractEulerAngles(const QMatrix4x4& matrix);
    QVector3D rotationMatrixToEulerAngles(const QMatrix3x3& R, const QString& order);
    void findFloorPlaneRANSAC();
    void runDbscanClustering();

    bool calculateRandomGraspPoseOnSegment(int targetSegmentId,
                                           const Eigen::Vector3f& mean,
                                           const Eigen::Vector3f& pc1,
                                           const Eigen::Vector3f& pc2,
                                           const Eigen::Vector3f& normal,
                                           QMatrix4x4& outPose,
                                           QVector3D& outPos_m,
                                           QVector3D& outOri_deg);

    bool checkPoseReachable(const QVector3D& pos_mm, const QVector3D& ori_deg);


    QHBoxLayout *m_layout;
    QLabel *m_colorLabel;
    PointCloudWidget *m_pointCloudWidget;
    QList<XYPlotWidget*> m_plotWidgets;
    HandlePlotWidget* m_handlePlotWidget;

    QMatrix4x4 m_baseToTcpTransform;
    QMatrix4x4 m_tcpToCameraTransform;

    rs2::pipeline m_pipeline; rs2::config m_config;
    rs2::pointcloud m_pointcloud; rs2::align m_align;
    rs2::decimation_filter m_dec_filter; rs2::spatial_filter m_spat_filter;
    rs2::disparity_transform m_depth_to_disparity, m_disparity_to_depth;
    bool m_isDenoisingOn = false;

    QTimer *m_timer, *m_resultTimer;
    QImage m_currentImage; cv::Mat m_latestFrame;
    rs2::points m_capturedPoints; QMatrix4x4 m_capturedBaseToTcpTransform;
    std::vector<int> m_uv_to_point_idx;

    int fd_image, fd_result, fd_control;
    void *data_image, *data_result, *data_control;
    sem_t *sem_image, *sem_result, *sem_control;

    QJsonArray m_detectionResults; bool m_isProcessing; bool m_showPlotWindow;
    QVector<GraspingTarget> m_graspingTargets;

    // PCA 정보
    Eigen::Vector3f m_pcaMean; Eigen::Vector3f m_pcaPC1; Eigen::Vector3f m_pcaPC2;
    Eigen::Vector3f m_pcaNormal;
    bool m_hasPCAData;
    QVector<QVector3D> m_handleCenterline3D;
    QVector<int> m_handleSegmentIds;

    QMatrix4x4 m_randomGraspPose; bool m_showRandomGraspPose;
    QMatrix4x4 m_icpGraspPose;
    bool m_showIcpGraspPose = false;
    QVector<QVector3D> m_selectedHandlePoints3D;
    QDialog* m_icpVizDialog = nullptr;
    PointCloudWidget* m_icpPointCloudWidget = nullptr;


    QMatrix4x4 m_calculatedTargetPose; QVector3D m_calculatedTargetPos_m; QVector3D m_calculatedTargetOri_deg;
    QMatrix4x4 m_calculatedTargetPose_Y_Aligned; QVector3D m_calculatedTargetOri_deg_Y_Aligned;
    QMatrix4x4 m_calculatedViewMatrix; QVector3D m_calculatedViewPos_mm; bool m_hasCalculatedViewPose;
    QElapsedTimer m_processingTimer;
    std::vector<int> m_clusterIds;

    const float APPROACH_HEIGHT_M = 0.15f;
    const float GRIPPER_Z_OFFSET = 0.146f;
    const int IMAGE_WIDTH = 640, IMAGE_HEIGHT = 480, IMAGE_CHANNELS = 3;
    const int IMAGE_SIZE = IMAGE_WIDTH * IMAGE_HEIGHT * IMAGE_CHANNELS;
    const int RESULT_SIZE = 100 * 1024, CONTROL_SIZE = 16;
    enum ControlOffset { OFFSET_NEW_FRAME = 0, OFFSET_RESULT_READY = 1, OFFSET_SHUTDOWN = 2 };
    bool m_isAutoSequenceCapture;
    bool m_newResultAwaitingFrameUpdate;
    QImage cvMatToQImage(const cv::Mat &mat);
    bool initSharedMemory();
    void sendImageToPython(const cv::Mat &mat);
    QJsonArray receiveResultsFromPython();
    void drawMaskOverlay(QImage &image, const QJsonArray &results);
};
#endif // REALSENSEWIDGET_H
