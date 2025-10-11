#include "realsensewidget.h"
#include <QDebug>
#include <QThread>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <semaphore.h>
#include <QJsonParseError>
#include <QPainterPath>
#include <QRegion>
#include <QPixmap>
#include <cmath>
#include <QKeyEvent>
#include <random>

// ===================================================================
// PointCloudWidget 구현
// ===================================================================

PointCloudWidget::PointCloudWidget(QWidget *parent)
    : QOpenGLWidget(parent)
{
    setFocusPolicy(Qt::StrongFocus);
    m_yaw = 0.0f;
    m_pitch = 0.0f;
    m_panX = 0.0f;
    m_panY = 0.0f;
    m_zoom = 1.0f;
}

PointCloudWidget::~PointCloudWidget() {}

void PointCloudWidget::updatePointCloud(const rs2::points& points, const rs2::video_frame& color, const QImage& maskOverlay)
{
    m_points = points;
    m_colorFrame = color;
    m_maskOverlay = maskOverlay;
    processPoints();
}

void PointCloudWidget::processPoints()
{
    if (!m_points || !m_colorFrame || m_points.size() == 0) {
        m_vertexData.clear();
        QMetaObject::invokeMethod(this, "update", Qt::QueuedConnection);
        return;
    }
    m_vertexData.clear();
    m_vertexData.reserve(m_points.size() * 6);
    const rs2::vertex* vertices = m_points.get_vertices();
    const rs2::texture_coordinate* tex_coords = m_points.get_texture_coordinates();
    const uchar* color_data = (const uchar*)m_colorFrame.get_data();
    int width = m_colorFrame.get_width();
    int height = m_colorFrame.get_height();
    bool useMask = !m_maskOverlay.isNull();

    for (size_t i = 0; i < m_points.size(); ++i) {
        if (m_isFloorFiltered && i < m_floorPoints.size() && m_floorPoints[i]) {
            continue;
        }
        if (vertices[i].z == 0) continue;
        int u = std::min(std::max(int(tex_coords[i].u * width + .5f), 0), width - 1);
        int v = std::min(std::max(int(tex_coords[i].v * height + .5f), 0), height - 1);
        bool isMasked = false;
        QRgb maskPixel = 0;
        if (useMask && m_maskOverlay.valid(u, v)) {
            maskPixel = m_maskOverlay.pixel(u, v);
            if (qAlpha(maskPixel) > 128) {
                isMasked = true;
            }
        }
        if (!m_showOnlyMaskedPoints || (m_showOnlyMaskedPoints && isMasked)) {
            m_vertexData.push_back(vertices[i].x);
            m_vertexData.push_back(vertices[i].y);
            m_vertexData.push_back(vertices[i].z);
            if (isMasked) {
                m_vertexData.push_back(qRed(maskPixel) / 255.0f);
                m_vertexData.push_back(qGreen(maskPixel) / 255.0f);
                m_vertexData.push_back(qBlue(maskPixel) / 255.0f);
            } else {
                int color_idx = (u + v * width) * 3;
                m_vertexData.push_back(color_data[color_idx + 2] / 255.0f);
                m_vertexData.push_back(color_data[color_idx + 1] / 255.0f);
                m_vertexData.push_back(color_data[color_idx] / 255.0f);
            }
        }
    }
    QMetaObject::invokeMethod(this, "update", Qt::QueuedConnection);
}

void PointCloudWidget::initializeGL()
{
    initializeOpenGLFunctions();
    glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glPointSize(1.0f);
}

void PointCloudWidget::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
}

void PointCloudWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float aspect = (float)width() / (height() > 0 ? height() : 1);
    const float hfov_rad = 69.4 * M_PI / 180.0;
    float view_width_at_1m = 2.0 * 1.0 * tan(hfov_rad / 2.0);
    float ortho_width = view_width_at_1m / 2.0;
    float ortho_height = ortho_width / aspect;
    glOrtho(-ortho_width, ortho_width, -ortho_height, ortho_height, -10.0, 10.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glScalef(m_zoom, m_zoom, 1.0f);
    glTranslatef(m_panX, m_panY, 0.0f);
    glRotatef(m_pitch, 1.0f, 0.0f, 0.0f);
    glRotatef(m_yaw, 0.0f, 1.0f, 0.0f);
    glRotatef(180, 1.0f, 0.0f, 0.0f);

    if (!m_vertexData.empty()) {
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_COLOR_ARRAY);
        int stride = 6 * sizeof(float);
        glVertexPointer(3, GL_FLOAT, stride, m_vertexData.data());
        glColorPointer(3, GL_FLOAT, stride, (char*)m_vertexData.data() + 3 * sizeof(float));
        glDrawArrays(GL_POINTS, 0, m_vertexData.size() / 6);
        glDisableClientState(GL_COLOR_ARRAY);
        glDisableClientState(GL_VERTEX_ARRAY);
    }
}

void PointCloudWidget::mousePressEvent(QMouseEvent *event)
{
    m_lastPos = event->pos();
}

void PointCloudWidget::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->position().x() - m_lastPos.x();
    int dy = event->position().y() - m_lastPos.y();
    if (event->buttons() & Qt::LeftButton) {
        m_yaw   += dx * 0.5f;
        m_pitch += dy * 0.5f;
    } else if (event->buttons() & Qt::RightButton) {
        m_panX += dx * 0.001f;
        m_panY -= dy * 0.001f;
    }
    m_lastPos = event->pos();
    update();
}

void PointCloudWidget::wheelEvent(QWheelEvent *event)
{
    if (event->angleDelta().y() > 0) {
        m_zoom *= 1.1f;
    } else {
        m_zoom *= 0.9f;
    }
    update();
}

void PointCloudWidget::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_1) {
        m_showOnlyMaskedPoints = !m_showOnlyMaskedPoints;
        qDebug() << "Show only masked points toggled to:" << m_showOnlyMaskedPoints;
        processPoints();
    } else if (event->key() == Qt::Key_2) {
        emit denoisingToggled();
    } else if (event->key() == Qt::Key_3) {
        emit floorRemovalToggled();
    } else {
        QOpenGLWidget::keyPressEvent(event);
    }
}

// ===================================================================
// RealSenseWidget 구현
// ===================================================================

const char* SHM_IMAGE_NAME = "realsense_image";
const char* SHM_RESULT_NAME = "detection_result";
const char* SHM_CONTROL_NAME = "detection_control";
const char* SEM_IMAGE_NAME = "/sem_realsense_image";
const char* SEM_RESULT_NAME = "/sem_detection_result";
const char* SEM_CONTROL_NAME = "/sem_detection_control";

RealSenseWidget::RealSenseWidget(QWidget *parent)
    : QWidget(parent), m_align(RS2_STREAM_COLOR), m_isProcessing(false)
{
    if (!initSharedMemory()) {
        qWarning() << "Shared memory initialization failed! Vision features will not work.";
    }
    m_dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    m_spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    m_spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5f);
    m_spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
    m_config.enable_stream(RS2_STREAM_DEPTH, IMAGE_WIDTH, IMAGE_HEIGHT, RS2_FORMAT_Z16, 30);
    m_config.enable_stream(RS2_STREAM_COLOR, IMAGE_WIDTH, IMAGE_HEIGHT, RS2_FORMAT_BGR8, 30);
    m_timer = new QTimer(this);
    connect(m_timer, &QTimer::timeout, this, &RealSenseWidget::updateFrame);
    m_resultTimer = new QTimer(this);
    connect(m_resultTimer, &QTimer::timeout, this, &RealSenseWidget::checkProcessingResult);
    m_layout = new QHBoxLayout(this);
    m_colorLabel = new QLabel(this);
    m_pointCloudWidget = new PointCloudWidget(this);
    connect(m_pointCloudWidget, &PointCloudWidget::denoisingToggled, this, &RealSenseWidget::onDenoisingToggled);
    connect(m_pointCloudWidget, &PointCloudWidget::floorRemovalToggled, this, &RealSenseWidget::onFloorRemovalToggled);
    m_colorLabel->setFrameShape(QFrame::Box);
    m_colorLabel->setAlignment(Qt::AlignCenter);
    m_colorLabel->setText("Waiting for Color Stream...");
    m_layout->setContentsMargins(0, 0, 0, 0);
    m_layout->addWidget(m_colorLabel, 1);
    m_layout->addWidget(m_pointCloudWidget, 1);
    this->setLayout(m_layout);
    QTimer::singleShot(500, this, &RealSenseWidget::startCameraStream);
}

RealSenseWidget::~RealSenseWidget()
{
    if (m_timer && m_timer->isActive()) m_timer->stop();
    if (m_resultTimer && m_resultTimer->isActive()) m_resultTimer->stop();
    if (fd_control != -1 && data_control != nullptr && sem_control != SEM_FAILED) {
        sem_wait(sem_control);
        static_cast<char*>(data_control)[OFFSET_SHUTDOWN] = 1;
        sem_post(sem_control);
    }
    if (data_image != nullptr) munmap(data_image, IMAGE_SIZE);
    if (fd_image != -1) { ::close(fd_image); shm_unlink(SHM_IMAGE_NAME); }
    if (sem_image != SEM_FAILED) { sem_close(sem_image); sem_unlink(SEM_IMAGE_NAME); }
    if (data_result != nullptr) munmap(data_result, RESULT_SIZE);
    if (fd_result != -1) { ::close(fd_result); shm_unlink(SHM_RESULT_NAME); }
    if (sem_result != SEM_FAILED) { sem_close(sem_result); sem_unlink(SEM_RESULT_NAME); }
    if (data_control != nullptr) munmap(data_control, CONTROL_SIZE);
    if (fd_control != -1) { ::close(fd_control); shm_unlink(SHM_CONTROL_NAME); }
    if (sem_control != SEM_FAILED) { sem_close(sem_control); sem_unlink(SEM_CONTROL_NAME); }
    try {
        m_pipeline.stop();
    } catch (const rs2::error &e) {
        qWarning() << "RealSense stop error:" << e.what();
    }
}

void RealSenseWidget::startCameraStream()
{
    try {
        m_pipeline.start(m_config);
        m_pipeline.wait_for_frames(5000);
        m_timer->start(33);
    } catch (const rs2::error &e) {
        qCritical() << "❌ RealSense Error:" << e.what();
    } catch (const std::exception &e) {
        qCritical() << "❌ Standard Error:" << e.what();
    }
}

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
    sendImageToPython(m_latestFrame);
    m_isProcessing = true;
    m_resultTimer->start(100);
}

void RealSenseWidget::checkProcessingResult()
{
    if (!m_isProcessing) {
        m_resultTimer->stop();
        return;
    }
    QJsonArray results = receiveResultsFromPython();
    if (!results.isEmpty()) {
        m_detectionResults = results;
        m_isProcessing = false;
        m_resultTimer->stop();
        qDebug() << "✅ Processing complete! Received" << results.size() << "detection results";
    }
}

void RealSenseWidget::onDenoisingToggled()
{
    m_isDenoisingOn = !m_isDenoisingOn;
    qDebug() << "Denoising filter toggled to:" << (m_isDenoisingOn ? "ON" : "OFF");
}

void RealSenseWidget::onFloorRemovalToggled()
{
    m_isFloorRemovalOn = !m_isFloorRemovalOn;
    qDebug() << "Floor removal toggled to:" << (m_isFloorRemovalOn ? "ON" : "OFF");
    if (m_isFloorRemovalOn) {
        findFloorPlaneRANSAC();
    } else {
        m_pointCloudWidget->m_isFloorFiltered = false;
        m_pointCloudWidget->processPoints();
    }
}

void RealSenseWidget::updateFrame()
{
    try {
        rs2::frameset frames = m_pipeline.wait_for_frames(1000);
        if (!frames) return;

        frames = m_align.process(frames);
        rs2::video_frame color_frame = frames.get_color_frame();
        rs2::depth_frame depth_frame = frames.get_depth_frame();

        if (!color_frame || !depth_frame) return;

        if (m_isDenoisingOn) {
            depth_frame = m_dec_filter.process(depth_frame);
            depth_frame = m_spat_filter.process(depth_frame);
        }

        m_pointcloud.map_to(color_frame);
        rs2::points points = m_pointcloud.calculate(depth_frame);

        cv::Mat color_mat(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3, (void*)color_frame.get_data(), color_frame.get_stride_in_bytes());
        m_latestFrame = color_mat.clone();
        m_currentImage = cvMatToQImage(color_mat);

        if (m_currentImage.isNull()) { // 유효성 검사 강화
             qWarning() << "Failed to convert cv::Mat to QImage.";
             return;
        }

        QImage maskOverlayImage(m_currentImage.size(), QImage::Format_ARGB32_Premultiplied);
        maskOverlayImage.fill(Qt::transparent);
        if (!m_detectionResults.isEmpty()) {
            drawMaskOverlay(m_currentImage, m_detectionResults);
            drawMaskOverlay(maskOverlayImage, m_detectionResults);
        }
        if (m_isProcessing) {
            QPainter painter(&m_currentImage);
            painter.setPen(Qt::yellow);
            painter.drawText(20, 30, "Processing...");
        }
        m_colorLabel->setPixmap(QPixmap::fromImage(m_currentImage).scaled(m_colorLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));

        m_pointCloudWidget->updatePointCloud(points, color_frame, maskOverlayImage);

        if (m_isFloorRemovalOn) {
            findFloorPlaneRANSAC();
        }
    } catch (const rs2::error &e) {
        qWarning() << "Frame update error:" << e.what();
    }
}

void RealSenseWidget::findFloorPlaneRANSAC()
{
    const rs2::points& points = m_pointCloudWidget->m_points;
    if (!points || points.size() < 100) {
        m_pointCloudWidget->m_isFloorFiltered = false;
        return;
    }
    const rs2::vertex* vertices = points.get_vertices();
    const size_t num_points = points.size();

    const int num_iterations = 100;
    const float distance_threshold = 0.01;

    std::vector<int> best_inliers_indices;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(0, num_points - 1);

    for (int i = 0; i < num_iterations; ++i) {
        int idx1 = distrib(gen);
        int idx2 = distrib(gen);
        int idx3 = distrib(gen);
        rs2::vertex p1 = vertices[idx1];
        rs2::vertex p2 = vertices[idx2];
        rs2::vertex p3 = vertices[idx3];
        float a = (p2.y - p1.y)*(p3.z - p1.z) - (p2.z - p1.z)*(p3.y - p1.y);
        float b = (p2.z - p1.z)*(p3.x - p1.x) - (p2.x - p1.x)*(p3.z - p1.z);
        float c = (p2.x - p1.x)*(p3.y - p1.y) - (p2.y - p1.y)*(p3.x - p1.x);
        float d = -(a*p1.x + b*p1.y + c*p1.z);
        float norm = sqrt(a*a + b*b + c*c);
        if (norm < 1e-6) continue;
        a /= norm; b /= norm; c /= norm; d /= norm;
        std::vector<int> current_inliers;
        for (size_t j = 0; j < num_points; ++j) {
            const rs2::vertex& p = vertices[j];
            if(p.x == 0 && p.y == 0 && p.z == 0) continue;
            float dist = std::abs(a*p.x + b*p.y + c*p.z + d);
            if (dist < distance_threshold) {
                current_inliers.push_back(j);
            }
        }
        if (current_inliers.size() > best_inliers_indices.size()) {
            best_inliers_indices = current_inliers;
        }
    }

    if (best_inliers_indices.size() > num_points / 4) {
        m_pointCloudWidget->m_floorPoints.assign(num_points, false);
        for (int idx : best_inliers_indices) {
            m_pointCloudWidget->m_floorPoints[idx] = true;
        }
        m_pointCloudWidget->m_isFloorFiltered = true;
    } else {
        m_pointCloudWidget->m_isFloorFiltered = false;
    }
    m_pointCloudWidget->processPoints();
}

QImage RealSenseWidget::cvMatToQImage(const cv::Mat &mat)
{
    if (mat.empty()) return QImage();
    if (mat.type() == CV_8UC3) {
        return QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_BGR888).rgbSwapped();
    }
    return QImage();
}

void RealSenseWidget::drawMaskOverlay(QImage &image, const QJsonArray &results)
{
    if (image.isNull()) return;
    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing, true);

    for (const QJsonValue &value : results) {
        QJsonObject cupResult = value.toObject();
        QStringList parts = {"body_handle", "body", "handle"};
        for (const QString &part : parts) {
            if (!cupResult.contains(part) || !cupResult[part].isObject()) continue;

            QJsonObject partData = cupResult[part].toObject();
            if (!(partData.contains("mask_rle") && partData.contains("mask_shape") && partData.contains("offset"))) continue;

            QJsonArray rleArray = partData["mask_rle"].toArray();
            QJsonArray shapeArray = partData["mask_shape"].toArray();
            QJsonArray offsetArray = partData["offset"].toArray();
            if (rleArray.isEmpty() || shapeArray.size() < 2 || offsetArray.size() < 2) continue;

            int H = shapeArray[0].toInt();
            int W = shapeArray[1].toInt();
            int maskOffsetX = offsetArray[0].toInt();
            int maskOffsetY = offsetArray[1].toInt();
            int cls_id = partData["cls_id"].toInt();

            QVector<uchar> mask_buffer(W * H, 0);
            int idx = 0;
            uchar val = 0;
            for(const QJsonValue& run_val : rleArray) {
                int len = run_val.toInt();
                int end = qMin(idx + len, W * H);
                if (end > idx) memset(mask_buffer.data() + idx, val, end - idx);
                idx = end;
                val = (val == 0 ? 255 : 0);
                if(idx >= W * H) break;
            }

            QImage mask_img(mask_buffer.constData(), W, H, W, QImage::Format_Alpha8);
            QColor maskColor;
            switch (cls_id) {
                case 0: maskColor = QColor(255, 0, 0, 150); break;
                case 1: maskColor = QColor(0, 255, 0, 150); break;
                case 2: maskColor = QColor(0, 0, 255, 150); break;
                default: maskColor = QColor(255, 255, 255, 150); break;
            }

            QImage colored(W, H, QImage::Format_ARGB32_Premultiplied);
            colored.fill(maskColor);
            QPainter p(&colored);
            p.setCompositionMode(QPainter::CompositionMode_DestinationIn);
            p.drawImage(0, 0, mask_img);
            p.end();

            painter.drawImage(maskOffsetX, maskOffsetY, colored);
        }
    }
}

bool RealSenseWidget::initSharedMemory()
{
    // 💡 수정: 기존 공유 메모리 파일이 있다면 삭제
    shm_unlink(SHM_IMAGE_NAME);
    shm_unlink(SHM_RESULT_NAME);
    shm_unlink(SHM_CONTROL_NAME);
    sem_unlink(SEM_IMAGE_NAME);
    sem_unlink(SEM_RESULT_NAME);
    sem_unlink(SEM_CONTROL_NAME);

    const int retryDelayMs = 500;
    auto setup_shm = [&](const char* name, int size, int& fd, void*& data) {
        while (true) {
            fd = shm_open(name, O_RDWR | O_CREAT, 0666);
            if (fd != -1) {
                if (ftruncate(fd, size) != -1) {
                    data = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
                    if (data != MAP_FAILED) return true;
                    qCritical() << "Failed to mmap" << name << ":" << strerror(errno);
                    ::close(fd); fd = -1;
                } else {
                    qCritical() << "Failed to ftruncate" << name << ":" << strerror(errno);
                    ::close(fd); fd = -1;
                }
            } else {
                qCritical() << "Failed to shm_open" << name << ":" << strerror(errno);
            }
            QThread::msleep(retryDelayMs);
        }
        return false;
    };

    if (!setup_shm(SHM_IMAGE_NAME, IMAGE_SIZE, fd_image, data_image)) return false;
    if (!setup_shm(SHM_RESULT_NAME, RESULT_SIZE, fd_result, data_result)) return false;
    if (!setup_shm(SHM_CONTROL_NAME, CONTROL_SIZE, fd_control, data_control)) return false;

    auto setup_sem = [&](const char* name, sem_t*& sem) {
        while (true) {
            sem = sem_open(name, O_CREAT, 0666, 1);
            if (sem != SEM_FAILED) return true;
            qCritical() << "Failed to open sem" << name << ":" << strerror(errno);
            QThread::msleep(retryDelayMs);
        }
        return false;
    };

    if (!setup_sem(SEM_IMAGE_NAME, sem_image)) return false;
    if (!setup_sem(SEM_RESULT_NAME, sem_result)) return false;
    if (!setup_sem(SEM_CONTROL_NAME, sem_control)) return false;

    sem_wait(sem_control);
    memset(data_control, 0, CONTROL_SIZE);
    sem_post(sem_control);

    return true;
}

void RealSenseWidget::sendImageToPython(const cv::Mat &mat)
{
    if (mat.empty() || data_image == nullptr || sem_image == SEM_FAILED || data_control == nullptr || sem_control == SEM_FAILED) return;

    sem_wait(sem_image);
    memcpy(data_image, mat.data, mat.total() * mat.elemSize());
    sem_post(sem_image);

    sem_wait(sem_control);
    static_cast<char*>(data_control)[OFFSET_NEW_FRAME] = 1;
    sem_post(sem_control);

    qDebug() << "✅ Image sent to Python for processing";
}

QJsonArray RealSenseWidget::receiveResultsFromPython()
{
    QJsonArray results;
    if (data_result == nullptr || sem_result == SEM_FAILED || data_control == nullptr || sem_control == SEM_FAILED) return results;

    sem_wait(sem_control);
    char resultReadyFlag = static_cast<char*>(data_control)[OFFSET_RESULT_READY];
    sem_post(sem_control);

    if (resultReadyFlag == 0) return results;

    sem_wait(sem_result);
    quint32 resultSize;
    memcpy(&resultSize, data_result, sizeof(quint32));
    if (resultSize > 0 && (int)resultSize <= RESULT_SIZE - (int)sizeof(quint32)) {
        QByteArray jsonData(static_cast<char*>(data_result) + sizeof(quint32), resultSize);
        QJsonDocument doc = QJsonDocument::fromJson(jsonData);
        if (doc.isArray()) {
            results = doc.array();
        }
    }
    sem_post(sem_result);

    sem_wait(sem_control);
    static_cast<char*>(data_control)[OFFSET_RESULT_READY] = 0;
    sem_post(sem_control);

    return results;
}
