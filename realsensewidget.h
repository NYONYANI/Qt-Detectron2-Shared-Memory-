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
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <vector> // 💡 vector 헤더 추가 확인
#include <sys/mman.h>
#include <semaphore.h>


class PointCloudWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:
    explicit PointCloudWidget(QWidget *parent = nullptr);
    ~PointCloudWidget();

    void updatePointCloud(const rs2::points& points, const rs2::video_frame& color, const QImage& maskOverlay);

signals:
    void denoisingToggled();
    void floorRemovalToggled(); // 💡 추가: 바닥 제거 토글 시그널

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

    std::vector<float> m_vertexData;

    rs2::points m_points;
    rs2::video_frame m_colorFrame = rs2::video_frame(nullptr);
    QImage m_maskOverlay;
    bool m_showOnlyMaskedPoints = false;

    // 💡 추가: 바닥 필터링 상태와 바닥 포인트 인덱스를 저장할 변수
    bool m_isFloorFiltered = false;
    std::vector<bool> m_floorPoints;
    friend class RealSenseWidget; // 💡 RealSenseWidget에서 접근 허용

    float m_yaw = 0.0f;
    float m_pitch = 0.0f;
    float m_zoom = 0.5f;
    float m_panX = 0.0f;
    float m_panY = 0.0f;
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

private slots:
    void onDenoisingToggled();
    void onFloorRemovalToggled(); // 💡 추가: 바닥 제거 슬롯
    void updateFrame();
    void checkProcessingResult();

private:
    // 💡 추가: 바닥 평면 감지를 위한 RANSAC 함수
    void findFloorPlaneRANSAC();

    QHBoxLayout *m_layout;
    QLabel *m_colorLabel;
    PointCloudWidget *m_pointCloudWidget;

    rs2::pipeline m_pipeline;
    rs2::config m_config;
    rs2::pointcloud m_pointcloud;
    rs2::align m_align;

    rs2::decimation_filter m_dec_filter;
    rs2::spatial_filter m_spat_filter;
    bool m_isDenoisingOn = false;
    bool m_isFloorRemovalOn = false; // 💡 추가: 바닥 제거 기능 활성화 플래그

    QTimer *m_timer;
    QTimer *m_resultTimer;
    QImage m_currentImage;
    cv::Mat m_latestFrame;

    int fd_image = -1;
    void* data_image = nullptr;
    sem_t* sem_image = SEM_FAILED;
    int fd_result = -1;
    void* data_result = nullptr;
    sem_t* sem_result = SEM_FAILED;
    int fd_control = -1;
    void* data_control = nullptr;
    sem_t* sem_control = SEM_FAILED;

    QJsonArray m_detectionResults;
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
