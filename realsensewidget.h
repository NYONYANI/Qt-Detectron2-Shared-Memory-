#ifndef REALSENSEWIDGET_H
#define REALSENSEWIDGET_H

#include <QWidget>
#include <QImage>
#include <QTimer>
#include <QPainter>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonDocument>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include <sys/mman.h>
#include <semaphore.h>

class RealSenseWidget : public QWidget
{
    Q_OBJECT

public:
    explicit RealSenseWidget(QWidget *parent = nullptr);
    ~RealSenseWidget();

public slots:
    void startCameraStream();
    void captureAndProcess(); // 💡 Capture 버튼 슬롯 추가

private slots:
    void updateFrame();
    void checkProcessingResult(); // 💡 결과 확인용 타이머 슬롯

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    // RealSense 멤버
    rs2::pipeline m_pipeline;
    rs2::config m_config;
    QTimer *m_timer;
    QTimer *m_resultTimer; // 💡 결과 확인용 타이머
    QImage m_currentImage;
    cv::Mat m_latestFrame; // 💡 최신 프레임 저장용

    // 공유 메모리 및 통신 관련 멤버
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
    bool m_isProcessing; // 💡 처리 중 플래그

    // 공유 메모리 크기 및 이름
    const int IMAGE_WIDTH = 640;
    const int IMAGE_HEIGHT = 480;
    const int IMAGE_CHANNELS = 3;
    const int IMAGE_SIZE = IMAGE_WIDTH * IMAGE_HEIGHT * IMAGE_CHANNELS;
    const int RESULT_SIZE = 100 * 1024;
    const int CONTROL_SIZE = 16;

    // 제어 플래그 오프셋
    enum ControlOffset {
        OFFSET_NEW_FRAME = 0,
        OFFSET_RESULT_READY = 1,
        OFFSET_SHUTDOWN = 2
    };

    // 헬퍼 함수
    QImage cvMatToQImage(const cv::Mat &mat);

    // 공유 메모리 헬퍼 함수
    bool initSharedMemory();
    void sendImageToPython(const cv::Mat &mat);
    QJsonArray receiveResultsFromPython();
    void drawMaskOverlay(QPainter &painter, const QJsonArray &results);
};

#endif // REALSENSEWIDGET_H
