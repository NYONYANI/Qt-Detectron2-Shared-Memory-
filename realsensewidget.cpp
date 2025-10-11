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
#include <GL/glu.h>

// ===================================================================
// PointCloudWidget 구현
// ===================================================================

PointCloudWidget::PointCloudWidget(QWidget *parent)
    : QOpenGLWidget(parent), m_colorFrame(nullptr)
{
    setFocusPolicy(Qt::StrongFocus);
    m_yaw = -45.0f;
    m_pitch = -30.0f;
    m_panX = 0.0f;
    m_panY = -0.3f;
    m_distance = 1.5f;
    m_baseToTcpTransform.setToIdentity();

    // Python 코드 기준 Hand-Eye Calibration 값 적용
    const float r[] = { 0.0f,  1.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f };
    QMatrix4x4 rotationMatrix(r);
    QMatrix4x4 translationMatrix;
    translationMatrix.translate(-0.080, 0.0325, 0.0308);
    m_tcpToCameraTransform = translationMatrix * rotationMatrix;
}

PointCloudWidget::~PointCloudWidget() {}

void PointCloudWidget::setTransforms(const QMatrix4x4& baseToTcp, const QMatrix4x4& tcpToCam)
{
    m_baseToTcpTransform = baseToTcp;
    m_tcpToCameraTransform = tcpToCam;
}

void PointCloudWidget::drawAxes(float length)
{
    glLineWidth(2.0);
    glBegin(GL_LINES);
    // X축 (Red)
    glColor3f(1.0f, 0.0f, 0.0f); glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(length, 0.0f, 0.0f);
    // Y축 (Green)
    glColor3f(0.0f, 1.0f, 0.0f); glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(0.0f, length, 0.0f);
    // Z축 (Blue)
    glColor3f(0.0f, 0.0f, 1.0f); glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(0.0f, 0.0f, length);
    glEnd();
    glLineWidth(1.0);
}

// ✨ [추가] 그리드 그리기 함수 구현
void PointCloudWidget::drawGrid(float size, int divisions)
{
    glLineWidth(1.0f);
    glColor3f(0.8f, 0.8f, 0.8f); // 연한 회색

    float step = size / divisions;
    float halfSize = size / 2.0f;

    glBegin(GL_LINES);
    for (int i = 0; i <= divisions; ++i) {
        float pos = -halfSize + i * step;
        // X축에 평행한 선 (Z=0 평면)
        glVertex3f(-halfSize, pos, 0.0f);
        glVertex3f(halfSize, pos, 0.0f);
        // Y축에 평행한 선 (Z=0 평면)
        glVertex3f(pos, -halfSize, 0.0f);
        glVertex3f(pos, halfSize, 0.0f);
    }
    glEnd();
}


void PointCloudWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (float)width() / (height() > 0 ? height() : 1), 0.1, 100.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // 카메라 시점 변환
    glTranslatef(0.0f, 0.0f, -m_distance);
    glTranslatef(m_panX, m_panY, 0.0f);
    glRotatef(m_pitch, 1.0f, 0.0f, 0.0f);
    glRotatef(m_yaw, 0.0f, 1.0f, 0.0f);

    // ✨ [추가] 로봇 베이스 평면에 그리드 그리기
    drawGrid(2.0f, 20); // 2x2 미터 크기, 10cm 간격

    drawAxes(0.2f); // 월드 좌표계 (로봇 베이스)

    glPushMatrix();
    glMultMatrixf(m_baseToTcpTransform.constData()); // 로봇 TCP 위치로 이동
    drawAxes(0.1f);

    glMultMatrixf(m_tcpToCameraTransform.constData()); // TCP -> 카메라 위치로 이동
    drawAxes(0.05f);

    if (!m_vertexData.empty()) {
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_COLOR_ARRAY);
        glVertexPointer(3, GL_FLOAT, 6 * sizeof(float), m_vertexData.data());
        glColorPointer(3, GL_FLOAT, 6 * sizeof(float), (char*)m_vertexData.data() + 3*sizeof(float));
        glDrawArrays(GL_POINTS, 0, m_vertexData.size() / 6);
        glDisableClientState(GL_COLOR_ARRAY);
        glDisableClientState(GL_VERTEX_ARRAY);
    }
    glPopMatrix();
}

void PointCloudWidget::initializeGL()
{
    initializeOpenGLFunctions();
    // ✨ [수정] 배경색을 흰색으로 변경
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glPointSize(1.5f);
}

void PointCloudWidget::resizeGL(int w, int h) { glViewport(0, 0, w, h>0 ? h : 1); }

void PointCloudWidget::updatePointCloud(const rs2::points& points, const rs2::video_frame& color, const QImage& maskOverlay) {
    m_points = points;
    m_colorFrame = color;
    m_maskOverlay = maskOverlay;
    processPoints();
}

void PointCloudWidget::processPoints() {
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
        if (m_isFloorFiltered && i < m_floorPoints.size() && m_floorPoints[i]) continue;
        if (vertices[i].z == 0) continue;

        int u = std::min(std::max(int(tex_coords[i].u * width + .5f), 0), width - 1);
        int v = std::min(std::max(int(tex_coords[i].v * height + .5f), 0), height - 1);

        bool isMasked = false;
        QRgb maskPixel = 0;
        if (useMask && m_maskOverlay.valid(u,v)) {
            maskPixel = m_maskOverlay.pixel(u,v);
            if (qAlpha(maskPixel) > 128) {
                isMasked = true;
            }
        }

        if (!m_showOnlyMaskedPoints || isMasked) {
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

void PointCloudWidget::mousePressEvent(QMouseEvent *event) { m_lastPos = event->pos(); }

void PointCloudWidget::mouseMoveEvent(QMouseEvent *event) {
    int dx = event->position().x() - m_lastPos.x(); int dy = event->position().y() - m_lastPos.y();
    if (event->buttons() & Qt::LeftButton) { m_yaw += dx * 0.5f; m_pitch += dy * 0.5f; }
    else if (event->buttons() & Qt::RightButton) { m_panX += dx * m_distance * 0.001f; m_panY -= dy * m_distance * 0.001f; }
    m_lastPos = event->pos(); update();
}

void PointCloudWidget::wheelEvent(QWheelEvent *event) {
    m_distance *= (event->angleDelta().y() > 0) ? 1.0f / 1.1f : 1.1f;
    if (m_distance < 0.1f) m_distance = 0.1f;
    if (m_distance > 10.0f) m_distance = 10.0f;
    update();
}

void PointCloudWidget::keyPressEvent(QKeyEvent *event) {
    if (event->key() == Qt::Key_1) {
        if (m_maskOverlay.isNull()) {
            qDebug() << "[WARN] Key '1' pressed, but no mask data available. Press 'Capture' first.";
            return;
        }
        m_showOnlyMaskedPoints = !m_showOnlyMaskedPoints;
        qDebug() << "[INFO] Show only masked points toggled to:" << m_showOnlyMaskedPoints;
        processPoints();
    }
    else if (event->key() == Qt::Key_2) emit denoisingToggled();
    else if (event->key() == Qt::Key_3) emit floorRemovalToggled();
    else if (event->key() == Qt::Key_4) emit showXYPlotRequested();
    else QOpenGLWidget::keyPressEvent(event);
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
    : QWidget(parent), m_align(RS2_STREAM_COLOR), m_isProcessing(false),
      m_depth_to_disparity(true), m_disparity_to_depth(false)
{
    initSharedMemory();
    m_config.enable_stream(RS2_STREAM_DEPTH, IMAGE_WIDTH, IMAGE_HEIGHT, RS2_FORMAT_Z16, 30);
    m_config.enable_stream(RS2_STREAM_COLOR, IMAGE_WIDTH, IMAGE_HEIGHT, RS2_FORMAT_BGR8, 30);
    m_timer = new QTimer(this); connect(m_timer, &QTimer::timeout, this, &RealSenseWidget::updateFrame);
    m_resultTimer = new QTimer(this); connect(m_resultTimer, &QTimer::timeout, this, &RealSenseWidget::checkProcessingResult);
    m_layout = new QHBoxLayout(this);
    m_colorLabel = new QLabel(this);
    m_pointCloudWidget = new PointCloudWidget(this);
    connect(m_pointCloudWidget, &PointCloudWidget::denoisingToggled, this, &RealSenseWidget::onDenoisingToggled);
    connect(m_pointCloudWidget, &PointCloudWidget::floorRemovalToggled, this, &RealSenseWidget::onFloorRemovalToggled);
    connect(m_pointCloudWidget, &PointCloudWidget::showXYPlotRequested, this, &RealSenseWidget::onShowXYPlot);
    m_layout->addWidget(m_colorLabel, 1); m_layout->addWidget(m_pointCloudWidget, 1); setLayout(m_layout);

    m_baseToTcpTransform.setToIdentity();
    const float r[] = { 0.0f,  1.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f };
    QMatrix4x4 rotationMatrix(r);
    QMatrix4x4 translationMatrix;
    translationMatrix.translate(-0.080, 0.0325, 0.0308);
    m_tcpToCameraTransform = translationMatrix * rotationMatrix;

    m_pointCloudWidget->setFocus();
    QTimer::singleShot(500, this, &RealSenseWidget::startCameraStream);
}

RealSenseWidget::~RealSenseWidget()
{
    if (m_timer && m_timer->isActive()) m_timer->stop();
    if (data_control) { static_cast<char*>(data_control)[OFFSET_SHUTDOWN] = 1; }
    qDeleteAll(m_plotWidgets);
    m_plotWidgets.clear();
    try { m_pipeline.stop(); } catch (...) {}
}

void RealSenseWidget::onRobotPoseUpdated(const float* pose)
{
    if (!pose) return;
    m_baseToTcpTransform.setToIdentity();
    m_baseToTcpTransform.translate(pose[0]/1000.0f, pose[1]/1000.0f, pose[2]/1000.0f);
    m_baseToTcpTransform.rotate(pose[5], 0, 0, 1);
    m_baseToTcpTransform.rotate(pose[4], 0, 1, 0);
    m_baseToTcpTransform.rotate(pose[3], 1, 0, 0);
}

void RealSenseWidget::onShowXYPlot()
{
    qDebug() << "[INFO] Key '4' pressed. Attempting to show XY plot...";
    if (m_detectedObjectsPoints.isEmpty()) {
        qDebug() << "[WARN] No detected objects to plot. Please press 'Capture' first and ensure objects are detected.";
        return;
    }

    for (int i = m_detectedObjectsPoints.size(); i < m_plotWidgets.size(); ++i) {
        m_plotWidgets[i]->hide();
    }

    for (int i = 0; i < m_detectedObjectsPoints.size(); ++i) {
        if (i >= m_plotWidgets.size()) {
            m_plotWidgets.append(new XYPlotWidget());
        }
        m_plotWidgets[i]->updateData(m_detectedObjectsPoints[i]);
        m_plotWidgets[i]->setWindowTitle(QString("Cup %1 Projection").arg(i + 1));
        m_plotWidgets[i]->show();
        m_plotWidgets[i]->activateWindow();
    }
}

void RealSenseWidget::updateFrame()
{
    try {
        rs2::frameset frames = m_pipeline.wait_for_frames(1000); if (!frames) return;
        frames = m_align.process(frames);
        rs2::video_frame color = frames.get_color_frame(); rs2::depth_frame depth = frames.get_depth_frame(); if (!color || !depth) return;

        if (m_isDenoisingOn) {
            depth = m_dec_filter.process(depth);
            depth = m_depth_to_disparity.process(depth);
            depth = m_spat_filter.process(depth);
            depth = m_disparity_to_depth.process(depth);
        }

        m_pointcloud.map_to(color);
        rs2::points points = m_pointcloud.calculate(depth);

        m_capturedPoints = points;
        m_capturedBaseToTcpTransform = m_baseToTcpTransform;


        m_pointCloudWidget->setTransforms(m_baseToTcpTransform, m_tcpToCameraTransform);

        QImage maskOverlayImage(color.get_width(), color.get_height(), QImage::Format_ARGB32_Premultiplied);
        maskOverlayImage.fill(Qt::transparent);
        if (!m_detectionResults.isEmpty()) {
            drawMaskOverlay(maskOverlayImage, m_detectionResults);
        }
        m_pointCloudWidget->updatePointCloud(points, color, maskOverlayImage);

        m_latestFrame = cv::Mat(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP).clone();
        m_currentImage = cvMatToQImage(m_latestFrame);

        if (!m_detectionResults.isEmpty()) {
            drawMaskOverlay(m_currentImage, m_detectionResults);
        }
        if(m_isProcessing) {
            QPainter painter(&m_currentImage);
            painter.setPen(Qt::yellow);
            painter.drawText(20, 30, "Processing...");
        }
        m_colorLabel->setPixmap(QPixmap::fromImage(m_currentImage).scaled(m_colorLabel->size(), Qt::KeepAspectRatio));

        m_pointCloudWidget->update();

    } catch (const rs2::error& e) {
        qWarning() << "RealSense error:" << e.what();
    } catch (...) {
        qWarning() << "Unknown error in updateFrame";
    }
}

void RealSenseWidget::startCameraStream() { try { m_pipeline.start(m_config); m_timer->start(33); } catch (const rs2::error &e) { qCritical() << "RealSense Error:" << e.what(); } }
QImage RealSenseWidget::cvMatToQImage(const cv::Mat &mat){ if(mat.empty()) return QImage(); return QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_BGR888).rgbSwapped(); }

void RealSenseWidget::captureAndProcess()
{
    if (m_isProcessing || m_latestFrame.empty()) {
        qDebug() << "[WARN] Capture failed. Is camera running? Or another process is already running?";
        return;
    }
    qDebug() << "[INFO] Capture button pressed. Sending frame to Python for processing...";

    sendImageToPython(m_latestFrame);
    m_isProcessing = true;
    m_resultTimer->start(100);
}

void RealSenseWidget::checkProcessingResult()
{
    if (!m_isProcessing) { m_resultTimer->stop(); return; }

    QJsonArray results = receiveResultsFromPython();
    if (!results.isEmpty()) {
        qDebug() << "[INFO] Received" << results.size() << "detection results from Python.";
        m_detectionResults = results;
        m_isProcessing = false;
        m_resultTimer->stop();

        m_detectedObjectsPoints.clear();
        if (!m_capturedPoints) {
            qWarning() << "[WARN] Captured points are invalid. Cannot create XY plot.";
            return;
        }

        const rs2::vertex* vertices = m_capturedPoints.get_vertices();
        const int width = IMAGE_WIDTH;
        QMatrix4x4 baseToCamTransform = m_capturedBaseToTcpTransform * m_tcpToCameraTransform;

        for (const QJsonValue &cupValue : m_detectionResults) {
            QJsonObject cupResult = cupValue.toObject();
            QVector<PlotData> singleCupPoints;
            QStringList parts = {"body", "handle"};
            for (const QString &part : parts) {
                if (!cupResult.contains(part) || !cupResult[part].isObject()) continue;

                QJsonObject partData = cupResult[part].toObject();
                QJsonArray rle = partData["mask_rle"].toArray();
                QJsonArray shape = partData["mask_shape"].toArray();
                QJsonArray offset = partData["offset"].toArray();
                int H = shape[0].toInt(); int W = shape[1].toInt();
                int ox = offset[0].toInt(); int oy = offset[1].toInt();
                int cls_id = partData["cls_id"].toInt();

                QVector<uchar> mask_buffer(W * H, 0);
                int idx = 0; uchar val = 0;
                for(const QJsonValue& run_val : rle) {
                    int len = run_val.toInt();
                    if(idx + len > W * H) len = W * H - idx;
                    if(len > 0) memset(mask_buffer.data() + idx, val, len);
                    idx += len;
                    val = (val == 0 ? 255 : 0);
                    if(idx >= W * H) break;
                }

                for(int y = 0; y < H; ++y) {
                    for(int x = 0; x < W; ++x) {
                        if(mask_buffer[y * W + x] == 255) {
                            int u = ox + x;
                            int v = oy + y;
                            if (u < 0 || u >= IMAGE_WIDTH || v < 0 || v >= IMAGE_HEIGHT) continue;

                            const rs2::vertex& p = vertices[v * width + u];
                            if (p.z > 0) {
                                QVector3D p_cam(p.x, p.y, p.z);
                                QVector3D p_base = baseToCamTransform * p_cam;
                                PlotData plotData;
                                plotData.point = QPointF(p_base.x(), p_base.y());
                                plotData.label = (cls_id == 1) ? "B" : "H";
                                plotData.color = (cls_id == 1) ? Qt::green : Qt::blue;
                                singleCupPoints.append(plotData);
                            }
                        }
                    }
                }
            }
            if(!singleCupPoints.isEmpty()) {
                m_detectedObjectsPoints.append(singleCupPoints);
            }
        }
    }
}


void RealSenseWidget::onDenoisingToggled() {
    m_isDenoisingOn = !m_isDenoisingOn;
    qDebug() << "[INFO] Denoising toggled:" << (m_isDenoisingOn ? "ON" : "OFF");
    updateFrame();
}
void RealSenseWidget::onFloorRemovalToggled() {
    m_isFloorRemovalOn = !m_isFloorRemovalOn;
    qDebug() << "[INFO] Floor removal toggled:" << (m_isFloorRemovalOn ? "ON" : "OFF");
    if (m_isFloorRemovalOn) findFloorPlaneRANSAC();
    else {
        m_pointCloudWidget->m_isFloorFiltered = false;
        m_pointCloudWidget->processPoints();
    }
}

void RealSenseWidget::findFloorPlaneRANSAC() {
    const rs2::points& points = m_pointCloudWidget->m_points;
    if (!points || points.size() < 100) {
        qDebug() << "[INFO] Floor removal: Not enough points to find floor.";
        m_pointCloudWidget->m_isFloorFiltered = false;
        return;
    }
    const rs2::vertex* vertices = points.get_vertices();
    const size_t num_points = points.size();
    std::vector<int> best_inliers_indices;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(0, num_points - 1);
    for (int i = 0; i < 100; ++i) {
        rs2::vertex p1 = vertices[distrib(gen)];
        rs2::vertex p2 = vertices[distrib(gen)];
        rs2::vertex p3 = vertices[distrib(gen)];
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
            if (std::abs(a*p.x + b*p.y + c*p.z + d) < 0.01) {
                current_inliers.push_back(j);
            }
        }
        if (current_inliers.size() > best_inliers_indices.size()) {
            best_inliers_indices = current_inliers;
        }
    }

    if (best_inliers_indices.size() > num_points / 4) {
        qDebug() << "[INFO] Floor removal: Floor plane found with" << best_inliers_indices.size() << "inliers.";
        m_pointCloudWidget->m_floorPoints.assign(num_points, false);
        for (int idx : best_inliers_indices) m_pointCloudWidget->m_floorPoints[idx] = true;
        m_pointCloudWidget->m_isFloorFiltered = true;
    } else {
        qDebug() << "[INFO] Floor removal: No dominant floor plane found. Best plane only had" << best_inliers_indices.size() << "inliers.";
        m_pointCloudWidget->m_isFloorFiltered = false;
    }
    m_pointCloudWidget->processPoints();
}

void RealSenseWidget::drawMaskOverlay(QImage &image, const QJsonArray &results) {
    if (image.isNull()) return;
    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing, true);
    int cupIndex = 1;
    for (const QJsonValue &value : results) {
        QJsonObject cupResult = value.toObject();
        QStringList parts = {"body", "handle"};
        for (const QString &part : parts) {
            if (!cupResult.contains(part) || !cupResult[part].isObject()) continue;

            QJsonObject partData = cupResult[part].toObject();
            if (!partData.contains("mask_rle") || !partData.contains("center")) continue;

            QJsonArray rle = partData["mask_rle"].toArray();
            QJsonArray shape = partData["mask_shape"].toArray();
            QJsonArray offset = partData["offset"].toArray();
            QJsonArray center = partData["center"].toArray();

            int H = shape[0].toInt();
            int W = shape[1].toInt();
            int ox = offset[0].toInt();
            int oy = offset[1].toInt();
            int centerX = center[0].toInt();
            int centerY = center[1].toInt();
            int cls = partData["cls_id"].toInt();

            QVector<uchar> mask_buffer(W * H, 0);
            int idx = 0; uchar val = 0;
            for(const QJsonValue& run_val : rle) {
                int len = run_val.toInt();
                if (idx + len > W*H) len = W*H - idx;
                if (len > 0) memset(mask_buffer.data() + idx, val, len);
                idx += len;
                val = (val == 0 ? 255 : 0);
                if(idx >= W * H) break;
            }

            QImage mask_img(mask_buffer.constData(), W, H, W, QImage::Format_Alpha8);
            QColor maskColor = (cls==1) ? QColor(0,255,0,150) : QColor(0,0,255,150);
            QImage colored(W, H, QImage::Format_ARGB32_Premultiplied);
            colored.fill(maskColor);
            QPainter p(&colored);
            p.setCompositionMode(QPainter::CompositionMode_DestinationIn);
            p.drawImage(0, 0, mask_img);
            p.end();
            painter.drawImage(ox, oy, colored);

            painter.setPen(Qt::white);
            painter.setFont(QFont("Arial", 12, QFont::Bold));
            QString label = QString("cup%1: %2").arg(cupIndex).arg(part);
            painter.drawText(centerX, centerY, label);
        }
        cupIndex++;
    }
}


bool RealSenseWidget::initSharedMemory() {
    shm_unlink(SHM_IMAGE_NAME); shm_unlink(SHM_RESULT_NAME); shm_unlink(SHM_CONTROL_NAME);
    sem_unlink(SEM_IMAGE_NAME); sem_unlink(SEM_RESULT_NAME); sem_unlink(SEM_CONTROL_NAME);
    auto setup_shm = [&](const char* name, int size, int& fd, void*& data) {
        fd = shm_open(name, O_RDWR | O_CREAT, 0666);
        if (fd != -1 && ftruncate(fd, size) != -1) {
            data = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
            if (data != MAP_FAILED) return true;
        }
        return false;
    };
    if (!setup_shm(SHM_IMAGE_NAME, IMAGE_SIZE, fd_image, data_image) ||
        !setup_shm(SHM_RESULT_NAME, RESULT_SIZE, fd_result, data_result) ||
        !setup_shm(SHM_CONTROL_NAME, CONTROL_SIZE, fd_control, data_control)) return false;
    auto setup_sem = [&](const char* name, sem_t*& sem) {
        sem = sem_open(name, O_CREAT, 0666, 1);
        return sem != SEM_FAILED;
    };
    if (!setup_sem(SEM_IMAGE_NAME, sem_image) ||
        !setup_sem(SEM_RESULT_NAME, sem_result) ||
        !setup_sem(SEM_CONTROL_NAME, sem_control)) return false;
    sem_wait(sem_control);
    memset(data_control, 0, CONTROL_SIZE);
    sem_post(sem_control);
    return true;
}

void RealSenseWidget::sendImageToPython(const cv::Mat &mat) {
    if (mat.empty() || !data_image || !sem_image || !data_control || !sem_control) return;
    sem_wait(sem_image);
    memcpy(data_image, mat.data, mat.total() * mat.elemSize());
    sem_post(sem_image);
    sem_wait(sem_control);
    static_cast<char*>(data_control)[OFFSET_NEW_FRAME] = 1;
    sem_post(sem_control);
}

QJsonArray RealSenseWidget::receiveResultsFromPython() {
    QJsonArray results;
    if (!data_result || !sem_result || !data_control || !sem_control) return results;
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
        if (doc.isArray()) results = doc.array();
    }
    sem_post(sem_result);
    sem_wait(sem_control);
    static_cast<char*>(data_control)[OFFSET_RESULT_READY] = 0;
    sem_post(sem_control);
    return results;
}
