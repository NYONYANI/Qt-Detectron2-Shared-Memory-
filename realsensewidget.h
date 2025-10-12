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

// 파지 지점의 위치와 방향 정보를 함께 저장하는 구조체
struct GraspingTarget
{
    QVector3D point;      // 3D 파지 포인트 (빨간 구의 중심)
    QVector3D direction;  // 파지 방향 (점선의 방향 벡터)
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
    void updateTargetPose(const QMatrix4x4& pose, bool show); // 목표 자세 업데이트 함수

signals:
    void denoisingToggled();
    void zFilterToggled();
    void showXYPlotRequested();
    void calculateTargetPoseRequested(); // 5번 키를 위한 시그널
    void moveRobotToPreGraspPoseRequested(); // 'm' 키를 위한 시그널

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
    void drawTargetPose(); // 목표 자세 그리기 함수 선언

    std::vector<float> m_vertexData;
    rs2::points m_points;
    rs2::video_frame m_colorFrame;
    QImage m_maskOverlay;
    bool m_showOnlyMaskedPoints = false;
    bool m_isZFiltered = false;
    bool m_isFloorFiltered = false;
    std::vector<bool> m_floorPoints;

    QVector<QVector3D> m_graspingPoints;
    QMatrix4x4 m_targetTcpTransform; // 목표 TCP 자세
    bool m_showTargetPose = false;   // 목표 자세 표시 여부 플래그

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
    void onRobotPoseUpdated(const float* pose);

signals:
    // QMatrix4x4를 직접 전달하도록 시그널 변경
    void requestRobotMove(const QMatrix4x4& poseMatrix);

private slots:
    void onDenoisingToggled();
    void onZFilterToggled();
    void onShowXYPlot();
    void onCalculateTargetPose();
    void onMoveRobotToPreGraspPose(); // 'm' 키를 위한 슬롯
    void updateFrame();
    void checkProcessingResult();

private:
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

    int fd_image, fd_result, fd_control;
    void *data_image, *data_result, *data_control;
    sem_t *sem_image, *sem_result, *sem_control;

    QJsonArray m_detectionResults;
    bool m_isProcessing;
    bool m_showPlotWindow;

    QVector<GraspingTarget> m_graspingTargets;
    QMatrix4x4 m_calculatedTargetPose; // 계산된 목표 자세를 저장할 변수
    std::vector<int> m_clusterIds;

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
