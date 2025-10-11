#include "realsensewidget.h"
#include <QDebug>
#include <QBuffer>
#include <QDataStream>
#include <QThread>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <semaphore.h>
#include <QTimer>
#include <QFile>
#include <QIODevice>
#include <QPainter>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QVector>
#include <cstring>

const char* SHM_IMAGE_NAME = "realsense_image";
const char* SHM_RESULT_NAME = "detection_result";
const char* SHM_CONTROL_NAME = "detection_control";

const char* SEM_IMAGE_NAME = "/sem_realsense_image";
const char* SEM_RESULT_NAME = "/sem_detection_result";
const char* SEM_CONTROL_NAME = "/sem_detection_control";

bool RealSenseWidget::initSharedMemory()
{
    const int retryDelayMs = 500;

    // 이미지 공유 메모리 연결
    while (true) {
        fd_image = shm_open(SHM_IMAGE_NAME, O_RDWR | O_CREAT, 0666);
        if (fd_image == -1) {
            qCritical() << "❌ Failed to shm_open SHM_IMAGE_NAME:" << strerror(errno) << "Retrying...";
            QThread::msleep(retryDelayMs);
            continue;
        }

        if (ftruncate(fd_image, IMAGE_SIZE) == -1) {
            qCritical() << "❌ Failed to ftruncate SHM_IMAGE_NAME:" << strerror(errno) << "Retrying...";
            ::close(fd_image);
            fd_image = -1;
            QThread::msleep(retryDelayMs);
            continue;
        }

        data_image = mmap(0, IMAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd_image, 0);
        if (data_image == MAP_FAILED) {
            qCritical() << "❌ Failed to mmap SHM_IMAGE_NAME:" << strerror(errno) << "Retrying...";
            ::close(fd_image);
            fd_image = -1;
            QThread::msleep(retryDelayMs);
            continue;
        }
        break;
    }

    // 결과 공유 메모리 연결
    while (true) {
        fd_result = shm_open(SHM_RESULT_NAME, O_RDWR | O_CREAT, 0666);
        if (fd_result == -1) {
            qCritical() << "❌ Failed to shm_open SHM_RESULT_NAME:" << strerror(errno) << "Retrying...";
            QThread::msleep(retryDelayMs);
            continue;
        }

        if (ftruncate(fd_result, RESULT_SIZE) == -1) {
            qCritical() << "❌ Failed to ftruncate SHM_RESULT_NAME:" << strerror(errno) << "Retrying...";
            ::close(fd_result);
            fd_result = -1;
            QThread::msleep(retryDelayMs);
            continue;
        }

        data_result = mmap(0, RESULT_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd_result, 0);
        if (data_result == MAP_FAILED) {
            qCritical() << "❌ Failed to mmap SHM_RESULT_NAME:" << strerror(errno) << "Retrying...";
            ::close(fd_result);
            fd_result = -1;
            QThread::msleep(retryDelayMs);
            continue;
        }
        break;
    }

    // 제어 공유 메모리 연결
    while (true) {
        fd_control = shm_open(SHM_CONTROL_NAME, O_RDWR | O_CREAT, 0666);
        if (fd_control == -1) {
            qCritical() << "❌ Failed to shm_open SHM_CONTROL_NAME:" << strerror(errno) << "Retrying...";
            QThread::msleep(retryDelayMs);
            continue;
        }

        if (ftruncate(fd_control, CONTROL_SIZE) == -1) {
            qCritical() << "❌ Failed to ftruncate SHM_CONTROL_NAME:" << strerror(errno) << "Retrying...";
            ::close(fd_control);
            fd_control = -1;
            QThread::msleep(retryDelayMs);
            continue;
        }

        data_control = mmap(0, CONTROL_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd_control, 0);
        if (data_control == MAP_FAILED) {
            qCritical() << "❌ Failed to mmap SHM_CONTROL_NAME:" << strerror(errno) << "Retrying...";
            ::close(fd_control);
            fd_control = -1;
            QThread::msleep(retryDelayMs);
            continue;
        }
        break;
    }

    // 세마포어 연결
    while (true) {
        sem_image = sem_open(SEM_IMAGE_NAME, O_CREAT, 0666, 1);
        if (sem_image == SEM_FAILED) {
            qCritical() << "❌ Failed to open sem_image:" << strerror(errno) << "Retrying...";
            QThread::msleep(retryDelayMs);
            continue;
        }
        break;
    }

    while (true) {
        sem_result = sem_open(SEM_RESULT_NAME, O_CREAT, 0666, 1);
        if (sem_result == SEM_FAILED) {
            qCritical() << "❌ Failed to open sem_result:" << strerror(errno) << "Retrying...";
            QThread::msleep(retryDelayMs);
            continue;
        }
        break;
    }

    while (true) {
        sem_control = sem_open(SEM_CONTROL_NAME, O_CREAT, 0666, 1);
        if (sem_control == SEM_FAILED) {
            qCritical() << "❌ Failed to open sem_control:" << strerror(errno) << "Retrying...";
            QThread::msleep(retryDelayMs);
            continue;
        }
        break;
    }

    // 제어 플래그 초기화
    sem_wait(sem_control);
    memset(data_control, 0, CONTROL_SIZE);
    sem_post(sem_control);

    return true;
}

void RealSenseWidget::sendImageToPython(const cv::Mat &mat)
{
    if (mat.empty()) {
        return;
    }
    if (fd_image == -1 || data_image == nullptr || sem_image == SEM_FAILED ||
        fd_control == -1 || data_control == nullptr || sem_control == SEM_FAILED) {
        return;
    }

    // 이미지 복사
    sem_wait(sem_image);
    try {
        size_t matSize = mat.total() * mat.elemSize();
        if (mat.data && matSize <= IMAGE_SIZE) {
            memcpy(data_image, mat.data, matSize);
        } else {
            qWarning() << "Image size mismatch or data is null. Mat size:" << matSize
                       << ", SHM size:" << IMAGE_SIZE;
        }
    } catch (...) {
        qCritical() << "Error while copying image to SHM.";
    }
    sem_post(sem_image);

    // 새 프레임 플래그 설정
    sem_wait(sem_control);
    static_cast<char*>(data_control)[OFFSET_NEW_FRAME] = 1;
    sem_post(sem_control);

    qDebug() << "✅ Image sent to Python for processing";
}

QJsonArray RealSenseWidget::receiveResultsFromPython()
{
    QJsonArray results;
    if (fd_result == -1 || data_result == nullptr || sem_result == SEM_FAILED ||
        fd_control == -1 || data_control == nullptr || sem_control == SEM_FAILED) {
        return results;
    }

    // 결과 준비 완료 플래그 확인
    sem_wait(sem_control);
    char resultReadyFlag = static_cast<char*>(data_control)[OFFSET_RESULT_READY];
    sem_post(sem_control);

    if (resultReadyFlag == 0) {
        return results;
    }

    // 결과 메모리 읽기
    sem_wait(sem_result);
    try {
        quint32 resultSize;
        if (RESULT_SIZE >= (int)sizeof(quint32)) {
            memcpy(&resultSize, data_result, sizeof(quint32));
        } else {
            qWarning() << "SHM Result too small to read size header. SHM size:" << RESULT_SIZE;
            sem_post(sem_result);
            return results;
        }

        if (resultSize > 0 && (int)resultSize <= RESULT_SIZE - (int)sizeof(quint32)) {
            QByteArray jsonData(static_cast<char*>(data_result) + sizeof(quint32), resultSize);
            QJsonParseError parseError;
            QJsonDocument doc = QJsonDocument::fromJson(jsonData, &parseError);

            if (parseError.error == QJsonParseError::NoError && doc.isArray()) {
                results = doc.array();
                qDebug() << "✅ Received detection results:" << results.size() << "objects";
            } else {
                qWarning() << "Failed to parse JSON result:" << parseError.errorString();
            }
        } else {
            qWarning() << "Invalid result size:" << resultSize << ", SHM size:" << RESULT_SIZE;
        }
    } catch (...) {
        qCritical() << "Error while reading results from SHM.";
    }
    sem_post(sem_result);

    // 결과 준비 완료 플래그 초기화
    sem_wait(sem_control);
    static_cast<char*>(data_control)[OFFSET_RESULT_READY] = 0;
    sem_post(sem_control);

    // 디버깅용: 결과를 저장하여 확인
    if (!results.isEmpty()) {
        QFile file("/tmp/realsense_results.json");
        if (file.open(QIODevice::WriteOnly)) {
            file.write(QJsonDocument(results).toJson());
            file.close();
        }
    }

    return results;
}

RealSenseWidget::~RealSenseWidget()
{
    if (m_timer->isActive()) {
        m_timer->stop();
    }
    if (m_resultTimer->isActive()) {
        m_resultTimer->stop();
    }

    // 제어 플래그 종료 신호
    if (fd_control != -1 && data_control != nullptr && sem_control != SEM_FAILED) {
        sem_wait(sem_control);
        static_cast<char*>(data_control)[OFFSET_SHUTDOWN] = 1;
        sem_post(sem_control);
    }

    // 메모리 매핑 해제 및 파일 디스크립터 닫기
    if (data_image != nullptr) {
        munmap(data_image, IMAGE_SIZE);
        data_image = nullptr;
    }
    if (fd_image != -1) {
        ::close(fd_image);
        shm_unlink(SHM_IMAGE_NAME);
        fd_image = -1;
    }
    if (sem_image != SEM_FAILED) {
        sem_close(sem_image);
        sem_unlink(SEM_IMAGE_NAME);
        sem_image = SEM_FAILED;
    }

    if (data_result != nullptr) {
        munmap(data_result, RESULT_SIZE);
        data_result = nullptr;
    }
    if (fd_result != -1) {
        ::close(fd_result);
        shm_unlink(SHM_RESULT_NAME);
        fd_result = -1;
    }
    if (sem_result != SEM_FAILED) {
        sem_close(sem_result);
        sem_unlink(SEM_RESULT_NAME);
        sem_result = SEM_FAILED;
    }

    if (data_control != nullptr) {
        munmap(data_control, CONTROL_SIZE);
        data_control = nullptr;
    }
    if (fd_control != -1) {
        ::close(fd_control);
        shm_unlink(SHM_CONTROL_NAME);
        fd_control = -1;
    }
    if (sem_control != SEM_FAILED) {
        sem_close(sem_control);
        sem_unlink(SEM_CONTROL_NAME);
        sem_control = SEM_FAILED;
    }

    try {
        m_pipeline.stop();
    } catch (const rs2::error &e) {
        qWarning() << "RealSense stop error:" << e.what();
    }
}

QImage RealSenseWidget::cvMatToQImage(const cv::Mat &mat)
{
    if (mat.empty()) {
        qWarning() << "cvMatToQImage: Mat is empty!";
        return QImage();
    }
    if (mat.type() == CV_8UC3) {
        cv::Mat rgb_mat;
        cv::cvtColor(mat, rgb_mat, cv::COLOR_BGR2RGB);
        return QImage(rgb_mat.data, rgb_mat.cols, rgb_mat.rows, rgb_mat.step, QImage::Format_RGB888).copy();
    }
    qWarning() << "cvMatToQImage: Mat type not CV_8UC3. Type:" << mat.type();
    return QImage();
}

void RealSenseWidget::drawMaskOverlay(QPainter &painter, const QJsonArray &results)
{
    QImage overlay(size(), QImage::Format_ARGB32_Premultiplied);
    overlay.fill(Qt::transparent);
    QPainter overlayPainter(&overlay);
    overlayPainter.setRenderHint(QPainter::Antialiasing, true);
    overlayPainter.setRenderHint(QPainter::SmoothPixmapTransform, true);

    // Compute scaling and offsets to match paintEvent's base image scaling
    float scaleX = (float)width() / IMAGE_WIDTH;
    float scaleY = (float)height() / IMAGE_HEIGHT;
    float scale = qMin(scaleX, scaleY); // Uniform scale to preserve aspect ratio
    int scaledImageWidth = (int)(IMAGE_WIDTH * scale);
    int scaledImageHeight = (int)(IMAGE_HEIGHT * scale);
    int offsetX = (width() - scaledImageWidth) / 2;  // Center horizontally
    int offsetY = (height() - scaledImageHeight) / 2; // Center vertically

    for (const QJsonValue &value : results) {
        if (!value.isObject()) {
            qWarning() << "Invalid result: not an object";
            continue;
        }
        QJsonObject cupResult = value.toObject();

        QStringList parts = {"body_handle", "body", "handle"};

        for (const QString &part : parts) {
            if (!cupResult.contains(part) || !cupResult[part].isObject()) {
                continue;
            }

            QJsonObject partData = cupResult[part].toObject();

            if (!(partData.contains("mask_rle") && partData.contains("mask_shape") && partData.contains("offset"))) {
                continue;
            }

            QJsonArray rleArray = partData["mask_rle"].toArray();
            QJsonArray shapeArray = partData["mask_shape"].toArray();
            QJsonArray offsetArray = partData["offset"].toArray();

            if (rleArray.isEmpty() || shapeArray.size() < 2 || offsetArray.size() < 2) {
                qWarning() << "Invalid mask or shape data for" << part;
                continue;
            }

            int H = shapeArray[0].toInt();
            int W = shapeArray[1].toInt();
            int maskOffsetX = offsetArray[0].toInt();
            int maskOffsetY = offsetArray[1].toInt();
            int cls_id = partData["cls_id"].toInt();

            // 1) RLE -> 1D 버퍼 복호화 (stride 문제 회피)
            QVector<uchar> mask(static_cast<int>(W * H), 0);
            int idx = 0;
            uchar valueAlpha = 0; // 필요 시 255로 시작해 반전 확인
            long long sumLen = 0;

            for (const QJsonValue &runLengthValue : rleArray) {
                int len = runLengthValue.toInt();
                if (len <= 0) {
                    continue;
                }
                sumLen += len;
                int end = qMin(idx + len, W * H);
                if (end > idx) {
                    memset(mask.data() + idx, valueAlpha, static_cast<size_t>(end - idx));
                }
                idx = end;
                valueAlpha = (valueAlpha == 0 ? 255 : 0);
                if (idx >= W * H) break;
            }
            if (idx != W * H) {
                qWarning() << "RLE length mismatch for" << part << ":" << idx << "!=" << (W * H)
                           << "(sum=" << sumLen << ")";
            }

            // 2) stride = W 인 Alpha8 QImage 생성 후 copy로 소유권 분리
            QImage maskImg(mask.constData(), W, H, W, QImage::Format_Alpha8);
            QImage maskCopy = maskImg.copy(); // mask 벡터 생명주기와 분리

            // 3) 색상을 먼저 채우고, 마스크를 알파로 적용 (빠르고 정확)
            QColor maskColor;
            switch (cls_id) {
                case 0: maskColor = QColor(255, 0, 0, 150);   break;
                case 1: maskColor = QColor(0, 255, 0, 150);   break;
                case 2: maskColor = QColor(0, 0, 255, 150);   break;
                default: maskColor = QColor(255, 255, 255, 150); break;
            }

            QImage colored(W, H, QImage::Format_ARGB32_Premultiplied);
            colored.fill(maskColor);

            {
                QPainter p(&colored);
                p.setRenderHint(QPainter::SmoothPixmapTransform, true);
                p.setCompositionMode(QPainter::CompositionMode_DestinationIn);
                p.drawImage(0, 0, maskCopy); // Alpha8을 알파 채널로 적용
            }

            // 4) 스케일과 오프셋 적용 (이미 scale을 고정 비율로 계산했으므로 추가 비율 보정 불필요)
            int drawW = qMax(1, (int)(W * scale));
            int drawH = qMax(1, (int)(H * scale));
            int drawX = (int)(maskOffsetX * scale) + offsetX;
            int drawY = (int)(maskOffsetY * scale) + offsetY;

            // drawImage(targetRect, image) 형태로 그리면 내부적으로 부드럽게 스케일링
            overlayPainter.drawImage(QRect(drawX, drawY, drawW, drawH), colored);

            // 중심점 표시
            if (partData.contains("center") && partData["center"].isArray()) {
                QJsonArray center = partData["center"].toArray();
                if (center.size() >= 2) {
                    QPoint scaledCenter(
                        (int)(center[0].toDouble() * scale + offsetX),
                        (int)(center[1].toDouble() * scale + offsetY)
                    );
                    overlayPainter.setPen(Qt::NoPen);
                    overlayPainter.setBrush(Qt::white);
                    overlayPainter.drawEllipse(scaledCenter, 3, 3);
                }
            }
        }
    }
    overlayPainter.end();

    painter.drawImage(0, 0, overlay);
}

RealSenseWidget::RealSenseWidget(QWidget *parent)
    : QWidget(parent), m_isProcessing(false)
{
    if (!initSharedMemory()) {
        qWarning() << "Shared memory initialization failed! Vision features will not work.";
    }
    m_config.enable_stream(RS2_STREAM_COLOR, IMAGE_WIDTH, IMAGE_HEIGHT, RS2_FORMAT_BGR8, 30);

    m_timer = new QTimer(this);
    connect(m_timer, &QTimer::timeout, this, &RealSenseWidget::updateFrame);

    // 💡 결과 확인용 타이머 (Capture 버튼 눌린 후 주기적으로 결과 확인)
    m_resultTimer = new QTimer(this);
    connect(m_resultTimer, &QTimer::timeout, this, &RealSenseWidget::checkProcessingResult);

    this->setMinimumSize(IMAGE_WIDTH, IMAGE_HEIGHT);

    QTimer::singleShot(500, this, &RealSenseWidget::startCameraStream);
}

void RealSenseWidget::startCameraStream()
{
    try {
        rs2::context ctx;
        auto devices = ctx.query_devices();
        if (devices.size() == 0) {
            qCritical() << "❌ No RealSense device found!";
            return;
        }

        m_pipeline.start(m_config);
        m_pipeline.wait_for_frames(5000);

        m_timer->start(33); // 약 30 FPS (화면 표시만)

    } catch (const rs2::error &e) {
        qCritical() << "❌ RealSense Error:" << e.what();
    } catch (const std::exception &e) {
        qCritical() << "❌ Standard Error:" << e.what();
    }
}

// 💡 Capture 버튼이 눌렸을 때 호출되는 슬롯
void RealSenseWidget::captureAndProcess()
{
    if (m_isProcessing) {
        qWarning() << "⚠️ Already processing a frame. Please wait...";
        return;
    }

    if (m_latestFrame.empty()) {
        qWarning() << "⚠️ No frame available to process";
        return;
    }

    qDebug() << "📸 Capture button pressed - sending frame to Python...";

    // 이미지를 Python으로 전송
    sendImageToPython(m_latestFrame);

    m_isProcessing = true;

    // 결과 확인 타이머 시작 (100ms마다 확인)
    m_resultTimer->start(100);
}

// 💡 결과가 준비되었는지 주기적으로 확인
void RealSenseWidget::checkProcessingResult()
{
    if (!m_isProcessing) {
        m_resultTimer->stop();
        return;
    }

    // Python으로부터 결과 수신 시도
    QJsonArray results = receiveResultsFromPython();

    if (!results.isEmpty()) {
        // 결과를 받았으면 저장하고 화면 갱신
        m_detectionResults = results;
        m_isProcessing = false;
        m_resultTimer->stop();

        qDebug() << "✅ Processing complete! Received" << results.size() << "detection results";

        // 화면 갱신
        this->update();
    }
}

// 💡 실시간 프레임 업데이트 (화면 표시만, Python 전송 X)
void RealSenseWidget::updateFrame()
{
    try {
        rs2::frameset frames = m_pipeline.wait_for_frames(1000);
        if (!frames) {
            qWarning() << "No frames received from RealSense.";
            return;
        }

        rs2::frame color_frame = frames.get_color_frame();
        if (!color_frame) {
            qWarning() << "No color frame received.";
            return;
        }

        rs2::video_frame video_frame = color_frame.as<rs2::video_frame>();
        if (!video_frame) {
            qWarning() << "Failed to cast color frame to video frame.";
            return;
        }

        cv::Mat color_mat(cv::Size(video_frame.get_width(), video_frame.get_height()),
                          CV_8UC3, (void*)video_frame.get_data(),
                          video_frame.get_stride_in_bytes());

        if (color_mat.empty()) {
            qWarning() << "Color mat is empty.";
            return;
        }

        // 💡 최신 프레임 저장 (Capture 버튼용)
        m_latestFrame = color_mat.clone();

        // 💡 화면 표시용 QImage 변환 (Python 전송 없음)
        m_currentImage = cvMatToQImage(color_mat);

        if (!m_currentImage.isNull()) {
            this->update(); // paintEvent 호출
        } else {
            qWarning() << "QImage conversion failed.";
        }
    } catch (const rs2::error &e) {
        qWarning() << "Frame update error:" << e.what();
        m_timer->stop();
    } catch (const std::exception &e) {
        qWarning() << "Standard exception in updateFrame:" << e.what();
    }
}

void RealSenseWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    QPainter painter(this);

    if (m_currentImage.isNull()) {
        painter.fillRect(rect(), Qt::black);
        painter.setPen(Qt::white);
        painter.drawText(rect(), Qt::AlignCenter, "Waiting for RealSense frames...");
        return;
    }

    QImage scaledImage = m_currentImage.scaled(size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
    int x = (width() - scaledImage.width()) / 2;
    int y = (height() - scaledImage.height()) / 2;

    painter.drawImage(x, y, scaledImage);

    // 💡 검출 결과가 있으면 오버레이 표시
    if (!m_detectionResults.isEmpty()) {
        drawMaskOverlay(painter, m_detectionResults);
    }

    // 💡 처리 중일 때 표시
    if (m_isProcessing) {
        painter.setPen(Qt::yellow);
        painter.drawText(10, 20, "Processing...");
    }
}
