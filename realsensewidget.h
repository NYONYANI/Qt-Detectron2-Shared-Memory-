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
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <sys/mman.h>
#include <semaphore.h>
#include "xyplotwidget.h"
#include <GL/glu.h>

class PointCloudWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:
    explicit PointCloudWidget(QWidget *parent = nullptr);
    ~PointCloudWidget();

    void updatePointCloud(const rs2::points& points, const rs2::video_frame& color, const QImage& maskOverlay);
    void setTransforms(const QMatrix4x4& baseToTcp, const QMatrix4x4& tcpToCam);

signals:
    void denoisingToggled();
    void floorRemovalToggled();
    void showXYPlotRequested();

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;

private:
    void processPoints();
    void drawAxes(float length = 0.1f);

    std::vector<float> m_vertexData;
    rs2::points m_points;
    rs2::video_frame m_colorFrame;
    QImage m_maskOverlay;
    bool m_showOnlyMaskedPoints = false;
    bool m_isFloorFiltered = false;
    std::vector<bool> m_floorPoints;
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

public slots:
    void startCameraStream();
    void captureAndProcess();
    void onRobotPoseUpdated(const float* pose);

private slots:
    void onDenoisingToggled();
    void onFloorRemovalToggled();
    void onShowXYPlot();
    void updateFrame();
    void checkProcessingResult();

private:
    void findFloorPlaneRANSAC();

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

    // ✨ [수정] Disparity Transform 필터 추가
    rs2::decimation_filter m_dec_filter;
    rs2::spatial_filter m_spat_filter;
    rs2::disparity_transform m_depth_to_disparity;
    rs2::disparity_transform m_disparity_to_depth;
    bool m_isDenoisingOn = false;
    bool m_isFloorRemovalOn = false;

    QTimer *m_timer;
    QTimer *m_resultTimer;
    QImage m_currentImage;
    cv::Mat m_latestFrame;

    int fd_image, fd_result, fd_control;
    void *data_image, *data_result, *data_control;
    sem_t *sem_image, *sem_result, *sem_control;

    QJsonArray m_detectionResults;
    QList<QVector<PlotData>> m_detectedObjectsPoints;
    bool m_isProcessing;

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
