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
#include <map>
#include <limits>
#include <QElapsedTimer> // ✨ [추가] 성능 측정을 위해 추가

// PointCloudWidget 구현 (변경 없음, 이전과 동일)
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

void PointCloudWidget::drawAxes(float length, float lineWidth)
{
    glLineWidth(lineWidth);
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 0.0f); glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(length, 0.0f, 0.0f);
    glColor3f(0.0f, 1.0f, 0.0f); glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(0.0f, length, 0.0f);
    glColor3f(0.0f, 0.0f, 1.0f); glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(0.0f, 0.0f, length);
    glEnd();
    glLineWidth(1.0);
}

void PointCloudWidget::drawGrid(float size, int divisions)
{
    glLineWidth(1.0f);
    glColor3f(0.8f, 0.8f, 0.8f);
    float step = size / divisions;
    float halfSize = size / 2.0f;
    glBegin(GL_LINES);
    for (int i = 0; i <= divisions; ++i) {
        float pos = -halfSize + i * step;
        glVertex3f(-halfSize, pos, 0.0f);
        glVertex3f(halfSize, pos, 0.0f);
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
    float aspect = (float)width() / (height() > 0 ? height() : 1);
    float view_size = m_distance * 0.5f;
    glOrtho(-view_size * aspect, view_size * aspect, -view_size, view_size, -100.0, 100.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(m_panX, m_panY, 0.0f);
    glRotatef(m_pitch, 1.0f, 0.0f, 0.0f);
    glRotatef(m_yaw, 0.0f, 1.0f, 0.0f);

    drawGrid(2.0f, 20);
    drawAxes(0.2f);
    drawGraspingSpheres();
    drawTargetPose();

    glPushMatrix();
    glMultMatrixf(m_baseToTcpTransform.constData());
    drawAxes(0.1f);
    drawGripper();
    glMultMatrixf(m_tcpToCameraTransform.constData());
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

void PointCloudWidget::processPoints(const std::vector<int>& clusterIds) {
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
    bool useClusters = !clusterIds.empty();

    QMatrix4x4 cameraToBaseTransform = m_baseToTcpTransform * m_tcpToCameraTransform;

    std::map<int, QVector3D> clusterColors;
    if (useClusters) {
        int maxClusterId = 0;
        for (int id : clusterIds) { if (id > maxClusterId) maxClusterId = id; }
        std::mt19937 gen(12345);
        std::uniform_real_distribution<float> distrib(0.0, 1.0);
        for (int i = 1; i <= maxClusterId; ++i) { clusterColors[i] = QVector3D(distrib(gen), distrib(gen), distrib(gen)); }
        clusterColors[-1] = QVector3D(0.5f, 0.5f, 0.5f);
    }

    for (size_t i = 0; i < m_points.size(); ++i) {
        if (vertices[i].z == 0) continue;

        if (m_isZFiltered) {
            QVector3D p_cam(vertices[i].x, vertices[i].y, vertices[i].z);
            QVector3D p_base = cameraToBaseTransform * p_cam;
            if (p_base.z() <= 0) continue;
        }

        if (m_isFloorFiltered && i < m_floorPoints.size() && m_floorPoints[i]) continue;

        if (!useClusters) {
            int u = std::min(std::max(int(tex_coords[i].u * width + .5f), 0), width - 1);
            int v = std::min(std::max(int(tex_coords[i].v * height + .5f), 0), height - 1);
            bool isMasked = false;
            QRgb maskPixel = 0;
            if (useMask && m_maskOverlay.valid(u,v)) {
                maskPixel = m_maskOverlay.pixel(u,v);
                if (qAlpha(maskPixel) > 128) isMasked = true;
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
        } else {
            if (i < clusterIds.size() && clusterIds[i] != 0) {
                m_vertexData.push_back(vertices[i].x);
                m_vertexData.push_back(vertices[i].y);
                m_vertexData.push_back(vertices[i].z);
                QVector3D color = clusterColors[clusterIds[i]];
                m_vertexData.push_back(color.x());
                m_vertexData.push_back(color.y());
                m_vertexData.push_back(color.z());
            }
        }
    }
    QMetaObject::invokeMethod(this, "update", Qt::QueuedConnection);
}

void PointCloudWidget::mousePressEvent(QMouseEvent *event) { m_lastPos = event->pos(); }

void PointCloudWidget::mouseMoveEvent(QMouseEvent *event) {
    int dx = event->position().x() - m_lastPos.x(); int dy = event->position().y() - m_lastPos.y();
    if (event->buttons() & Qt::LeftButton) {
        m_yaw += dx * 0.5f;
        m_pitch += dy * 0.5f;
    }
    else if (event->buttons() & Qt::RightButton) {
        m_panX += dx * m_distance * 0.001f;
        m_panY -= dy * m_distance * 0.001f;
    }
    m_lastPos = event->pos();
    update();
}

void PointCloudWidget::wheelEvent(QWheelEvent *event) {
    if (event->angleDelta().y() > 0) m_distance *= 1.0f / 1.1f;
    else m_distance *= 1.1f;
    if (m_distance < 0.01f) m_distance = 0.01f;
    if (m_distance > 50.0f) m_distance = 50.0f;
    update();
}

void PointCloudWidget::keyPressEvent(QKeyEvent *event) {
    if (event->key() == Qt::Key_1) {
        if (m_maskOverlay.isNull()) {
            qDebug() << "[WARN] Key '1' pressed, but no mask data available."; return;
        }
        m_showOnlyMaskedPoints = !m_showOnlyMaskedPoints;
        qDebug() << "[INFO] Show only masked points toggled to:" << m_showOnlyMaskedPoints;
        processPoints();
    }
    else if (event->key() == Qt::Key_2) emit denoisingToggled();
    else if (event->key() == Qt::Key_3) emit zFilterToggled();
    else if (event->key() == Qt::Key_4) emit showXYPlotRequested();
    else if (event->key() == Qt::Key_5) emit calculateTargetPoseRequested();
    else if (event->key() == Qt::Key_M) emit moveRobotToPreGraspPoseRequested();
    else if (event->key() == Qt::Key_D) emit pickAndReturnRequested(); // ✨ [추가] 'D' 키 시그널 발생
    else QOpenGLWidget::keyPressEvent(event);
}

void PointCloudWidget::updateGraspingPoints(const QVector<QVector3D> &points) {
    m_graspingPoints = points;
    update();
}

void PointCloudWidget::drawGraspingSpheres() {
    if (m_graspingPoints.isEmpty()) return;
    GLUquadric* quadric = gluNewQuadric();
    gluQuadricNormals(quadric, GLU_SMOOTH);
    glColor3f(1.0f, 0.0f, 0.0f);
    for (const auto& point : m_graspingPoints) {
        glPushMatrix();
        glTranslatef(point.x(), point.y(), point.z());
        gluSphere(quadric, 0.005, 16, 16);
        glPopMatrix();
    }
    gluDeleteQuadric(quadric);
}

void PointCloudWidget::drawGripper() {
    const float gripper_z_offset = 0.146f;
    const float gripper_half_width = 0.040f;
    const float gripper_jaw_length = 0.05f;
    glLineWidth(3.0f);
    glColor3f(0.0f, 0.0f, 0.0f);
    glBegin(GL_LINES);
    glVertex3f(-gripper_jaw_length / 2, gripper_half_width, gripper_z_offset);
    glVertex3f( gripper_jaw_length / 2, gripper_half_width, gripper_z_offset);
    glVertex3f(-gripper_jaw_length / 2, -gripper_half_width, gripper_z_offset);
    glVertex3f( gripper_jaw_length / 2, -gripper_half_width, gripper_z_offset);
    glEnd();
    glLineWidth(1.0f);
}

void PointCloudWidget::drawTargetPose()
{
    if (!m_showTargetPose) return;
    glPushMatrix();
    glMultMatrixf(m_targetTcpTransform.constData());
    glPushAttrib(GL_CURRENT_BIT);
    glColor3f(0.5f, 0.0f, 0.8f);
    drawAxes(0.1f, 4.0f);
    glPopAttrib();
    glPopMatrix();
}

void PointCloudWidget::updateTargetPose(const QMatrix4x4 &pose, bool show)
{
    m_targetTcpTransform = pose;
    m_showTargetPose = show;
    update();
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
    m_showPlotWindow(false),
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
    connect(m_pointCloudWidget, &PointCloudWidget::zFilterToggled, this, &RealSenseWidget::onZFilterToggled);
    connect(m_pointCloudWidget, &PointCloudWidget::showXYPlotRequested, this, &RealSenseWidget::onShowXYPlot);
    connect(m_pointCloudWidget, &PointCloudWidget::calculateTargetPoseRequested, this, &RealSenseWidget::onCalculateTargetPose);
    connect(m_pointCloudWidget, &PointCloudWidget::moveRobotToPreGraspPoseRequested, this, &RealSenseWidget::onMoveRobotToPreGraspPose);
    connect(m_pointCloudWidget, &PointCloudWidget::pickAndReturnRequested, this, &RealSenseWidget::onPickAndReturnRequested); // ✨ [추가] D 키 연결
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

void RealSenseWidget::setShowPlot(bool show)
{
    m_showPlotWindow = show;
}

void RealSenseWidget::onRobotTransformUpdated(const QMatrix4x4 &transform)
{
    m_baseToTcpTransform = transform;
}

void RealSenseWidget::onShowXYPlot()
{
    QElapsedTimer timer;
    timer.start();
    qDebug() << "[INFO] Key '4' pressed. Calculating grasping points...";
    if (m_detectionResults.isEmpty() || !m_pointCloudWidget->m_points) {
        qDebug() << "[WARN] No data available for calculation."; return;
    }
    const rs2::points& currentPoints = m_pointCloudWidget->m_points;
    const rs2::vertex* vertices = currentPoints.get_vertices();
    const int width = IMAGE_WIDTH; const int height = IMAGE_HEIGHT;
    QMatrix4x4 camToBaseTransform = m_baseToTcpTransform * m_tcpToCameraTransform;

    m_graspingTargets.clear();
    QVector<QVector3D> graspingPointsForViz;

    QList<QVector<PlotData>> detectedBodyPoints;
    QList<QVector<PlotData>> detectedHandlePoints;

    for (const QJsonValue &cupValue : m_detectionResults) {
        QJsonObject cupResult = cupValue.toObject();
        QVector<QVector3D> singleCupBodyPoints3D;
        QVector<QVector3D> singleCupHandlePoints3D;

        QStringList parts = {"body", "handle"};
        for (const QString &part : parts) {
            if (!cupResult.contains(part) || !cupResult[part].isObject()) continue;
            QJsonObject partData = cupResult[part].toObject();
            QJsonArray rle = partData["mask_rle"].toArray();
            QJsonArray shape = partData["mask_shape"].toArray();
            QJsonArray offset = partData["offset"].toArray();
            int H = shape[0].toInt(); int W = shape[1].toInt();
            int ox = offset[0].toInt(); int oy = offset[1].toInt();
            QVector<uchar> mask_buffer(W * H, 0);
            int idx = 0; uchar val = 0;
            for(const QJsonValue& run_val : rle) {
                int len = run_val.toInt();
                if(idx + len > W * H) len = W * H - idx;
                if(len > 0) memset(mask_buffer.data() + idx, val, len);
                idx += len; val = (val == 0 ? 255 : 0);
                if(idx >= W * H) break;
            }
            for(int y = 0; y < H; ++y) {
                for(int x = 0; x < W; ++x) {
                    if(mask_buffer[y * W + x] == 255) {
                        int u_mask = ox + x;
                        int v_mask = oy + y;
                        if (u_mask < 0 || u_mask >= width || v_mask < 0 || v_mask >= height) continue;

                        // ✨ [개선] 조회 테이블을 사용한 빠른 3D 포인트 검색
                        int point_idx = m_uv_to_point_idx[v_mask * width + u_mask];
                        if (point_idx != -1) {
                            const rs2::vertex& p = vertices[point_idx];
                            if (p.z > 0) {
                                QVector3D p_cam(p.x, p.y, p.z);
                                QVector3D p_base = camToBaseTransform * p_cam;
                                if (m_pointCloudWidget->m_isZFiltered && p_base.z() <= 0) continue;
                                if (part == "body") singleCupBodyPoints3D.append(p_base);
                                else if (part == "handle") singleCupHandlePoints3D.append(p_base);
                            }
                        }
                    }
                }
            }
        }

        if(singleCupBodyPoints3D.isEmpty()) continue;

        QVector<QPointF> bodyPoints2D;
        float max_z = -std::numeric_limits<float>::infinity();
        for(const auto& p3d : singleCupBodyPoints3D) {
            bodyPoints2D.append(p3d.toPointF());
            if (p3d.z() > max_z) max_z = p3d.z();
        }

        CircleResult circle = CircleFitter::fitCircleLeastSquares(bodyPoints2D);
        if (circle.radius > 0 && circle.radius < 1.0) {
            float grasp_z = max_z - 0.01f;
            if (!singleCupHandlePoints3D.isEmpty()){
                double h_sum_x = 0, h_sum_y = 0;
                for(const auto& p3d : singleCupHandlePoints3D) { h_sum_x += p3d.x(); h_sum_y += p3d.y(); }
                QPointF handleCentroid(h_sum_x/singleCupHandlePoints3D.size(), h_sum_y/singleCupHandlePoints3D.size());
                QPointF circleCenter(circle.centerX, circle.centerY);

                QLineF fittedLine(circleCenter, handleCentroid);

                QLineF perpLine(0, 0, -fittedLine.dy(), fittedLine.dx());
                QVector3D perpDir(perpLine.dx(), perpLine.dy(), 0);
                perpDir.normalize();

                QVector3D graspPoint1(
                    circleCenter.x() + circle.radius * perpDir.x(),
                    circleCenter.y() + circle.radius * perpDir.y(),
                    grasp_z
                    );
                m_graspingTargets.append({graspPoint1, perpDir, circleCenter, handleCentroid});
                graspingPointsForViz.append(graspPoint1);

                QVector3D graspPoint2(
                    circleCenter.x() - circle.radius * perpDir.x(),
                    circleCenter.y() - circle.radius * perpDir.y(),
                    grasp_z
                    );
                m_graspingTargets.append({graspPoint2, -perpDir, circleCenter, handleCentroid});
                graspingPointsForViz.append(graspPoint2);
            }
        }

        if(m_showPlotWindow) {
            QVector<PlotData> bodyPlotData, handlePlotData;
            for(const auto& p3d : singleCupBodyPoints3D) bodyPlotData.append({p3d.toPointF(), Qt::green, "B"});
            for(const auto& p3d : singleCupHandlePoints3D) handlePlotData.append({p3d.toPointF(), Qt::blue, "H"});
            detectedBodyPoints.append(bodyPlotData);
            detectedHandlePoints.append(handlePlotData);
        }
    }

    m_pointCloudWidget->updateGraspingPoints(graspingPointsForViz);
    qDebug() << "[INFO] Grasping point calculation finished in" << timer.elapsed() << "ms.";


    if (m_showPlotWindow && !detectedBodyPoints.isEmpty()) {
        for (int i = detectedBodyPoints.size(); i < m_plotWidgets.size(); ++i) m_plotWidgets[i]->hide();
        for (int i = 0; i < detectedBodyPoints.size(); ++i) {
            if (i >= m_plotWidgets.size()) m_plotWidgets.append(new XYPlotWidget());
            m_plotWidgets[i]->updateData(detectedBodyPoints[i], detectedHandlePoints[i]);
            m_plotWidgets[i]->setWindowTitle(QString("Cup %1 Fitting Result").arg(i + 1));
            m_plotWidgets[i]->show();
            m_plotWidgets[i]->activateWindow();
        }
    } else if (m_showPlotWindow) {
        qDebug() << "[WARN] No body points found for plotting.";
    }
}

void RealSenseWidget::onCalculateTargetPose()
{
    qDebug() << "[INFO] Key '5' pressed. Calculating target pose...";
    if (m_graspingTargets.isEmpty()) {
        qDebug() << "[WARN] No grasping points calculated yet. Press '4' first.";
        m_pointCloudWidget->updateTargetPose(QMatrix4x4(), false);
        return;
    }

    // 가장 가까운 파지점을 선택
    QVector3D currentTcpPos = m_baseToTcpTransform.column(3).toVector3D();
    float minDistance = std::numeric_limits<float>::max();
    GraspingTarget bestTarget;

    for (const auto& target : m_graspingTargets) {
        float distance = currentTcpPos.distanceToPoint(target.point);
        if (distance < minDistance) {
            minDistance = distance;
            bestTarget = target;
        }
    }

    // bestTarget.direction은 원 중심을 향하는 법선
    // 손잡이는 법선 반대 방향에 있음
    QVector3D toHandle = -bestTarget.direction; // 원 중심에서 손잡이로 향하는 방향

    // 1. 목표 위치(미터 단위)를 계산합니다.
    const float gripper_z_offset = 0.146f;
    m_calculatedTargetPos_m = bestTarget.point + QVector3D(0, 0, gripper_z_offset);

    // 2. 파지 방향(손잡이 방향)으로부터 Rz(Yaw) 각도를 계산합니다.
    const QVector3D& N = bestTarget.direction; // 원 중심을 향하는 법선 벡터

    // ✨ [수정] Y축 방향 결정: +Y 또는 -Y 중 -X축이 손잡이 방향을 더 잘 보도록

    // Case 1: -Y축이 원의 중심을 향하는 경우 (Rz1)
    float rz1 = atan2(N.x(), N.y());

    // Case 2: +Y축이 원의 중심을 향하는 경우 (Rz2 = Rz1 + 180도)
    float rz2 = rz1 + M_PI;

    // 각 경우의 X축 방향을 계산
    // Rz 회전 후 X축 방향은: X_rotated = [cos(Rz), -sin(Rz), 0]
    QVector3D xAxis1(cos(rz1), -sin(rz1), 0);
    QVector3D xAxis2(cos(rz2), -sin(rz2), 0);

    // 손잡이 방향과의 내적을 계산하여 더 잘 정렬된 것을 선택
    // (내적이 음수이고 작을수록 -X축이 손잡이를 향함)
    float dot1 = QVector3D::dotProduct(xAxis1, toHandle);
    float dot2 = QVector3D::dotProduct(xAxis2, toHandle);

    float target_rz_rad;
    if (dot1 < dot2) {
        // Case 1이 -X축이 손잡이에 더 가까움
        target_rz_rad = rz1;
        qDebug() << "[INFO] Selected -Y towards center (Case 1), -X-axis towards handle (dot=" << dot1 << ")";
    } else {
        // Case 2가 -X축이 손잡이에 더 가까움
        target_rz_rad = rz2;
        qDebug() << "[INFO] Selected +Y towards center (Case 2), -X-axis towards handle (dot=" << dot2 << ")";
    }

    // 각도를 -π ~ π 범위로 정규화
    while (target_rz_rad > M_PI) target_rz_rad -= 2 * M_PI;
    while (target_rz_rad < -M_PI) target_rz_rad += 2 * M_PI;

    // 3. 사용자가 요청한 고정 각도를 설정합니다. (Rx=0, Ry=180)
    float target_rx = 0.0f;
    float target_ry = 179.9f; // 특이점을 피하기 위해 180 대신 179.9 사용

    // 4. 계산된 오일러 각도를 멤버 변수에 저장합니다.
    m_calculatedTargetOri_deg = QVector3D(
        target_rx,
        target_ry,
        qRadiansToDegrees(target_rz_rad)
        );

    // 5. 시각화를 위해 오일러 각도로부터 4x4 행렬을 생성합니다.
    QMatrix4x4 vizMatrix;
    vizMatrix.setToIdentity();
    vizMatrix.translate(m_calculatedTargetPos_m);

    vizMatrix.rotate(m_calculatedTargetOri_deg.x(), 1, 0, 0); // Roll (Rx)
    vizMatrix.rotate(m_calculatedTargetOri_deg.y(), 0, 1, 0); // Pitch (Ry)
    vizMatrix.rotate(m_calculatedTargetOri_deg.z(), 0, 0, 1); // Yaw (Rz)

    m_calculatedTargetPose = vizMatrix;

    qDebug() << "[TARGET POSE] Calculated Target Orientation (Rx, Ry, Rz deg):"
             << m_calculatedTargetOri_deg.x()
             << m_calculatedTargetOri_deg.y()
             << m_calculatedTargetOri_deg.z();

    qDebug() << "Target Pose Calculated: Pos(m):" << m_calculatedTargetPos_m << "Ori(deg):" << m_calculatedTargetOri_deg;

    m_pointCloudWidget->updateTargetPose(m_calculatedTargetPose, !m_pointCloudWidget->m_showTargetPose);
}

// RealSenseWidget::onMoveRobotToPreGraspPose (M 키 슬롯)
void RealSenseWidget::onMoveRobotToPreGraspPose()
{
    qDebug() << "[INFO] Key 'M' pressed. Requesting robot move to pre-grasp and gripper open...";
    if (m_calculatedTargetPos_m.isNull() || m_calculatedTargetOri_deg.isNull()) {
        qWarning() << "[WARN] No target pose has been calculated. Press '5' first.";
        return;
    }

    // Pre-grasp 위치 계산 (Z축으로 15cm 위)
    QVector3D preGraspPos_m = m_calculatedTargetPos_m + QVector3D(0, 0, APPROACH_HEIGHT_M); // 0.15m 위

    QVector3D preGraspPos_mm = preGraspPos_m * 1000.0f;
    QVector3D preGraspOri_deg = m_calculatedTargetOri_deg;

    qDebug() << "Requesting move to Pre-Grasp Pose: Pos(mm):" << preGraspPos_mm << "Ori(deg):" << preGraspOri_deg;

    // ✨ [수정] 그리퍼 열기 시퀀스 요청 (이동 전에 수행)
    emit requestGripperAction(0); // 0: Open

    // 계산된 위치(mm)와 각도(deg)를 로봇 이동 시그널로 전달
    emit requestRobotMove(preGraspPos_mm, preGraspOri_deg);
}


// ✨ [추가] RealSenseWidget::onPickAndReturnRequested (D 키 슬롯)
void RealSenseWidget::onPickAndReturnRequested()
{
    qDebug() << "[INFO] Key 'D' pressed. Requesting pick and return sequence...";

    if (m_calculatedTargetPos_m.isNull() || m_calculatedTargetOri_deg.isNull()) {
        qWarning() << "[WARN] Target pose not ready. Press '5' first.";
        return;
    }

    // 1. 최종 파지 자세 (mm)
    QVector3D finalTargetPos_m = m_calculatedTargetPos_m;
    QVector3D finalTargetPos_mm = finalTargetPos_m * 1000.0f;
    QVector3D finalTargetOri_deg = m_calculatedTargetOri_deg;

    // 2. 복귀/접근 자세 (mm) (M 키로 이동했던 위치)
    QVector3D approachPos_m = m_calculatedTargetPos_m + QVector3D(0, 0, APPROACH_HEIGHT_M);
    QVector3D approachPos_mm = approachPos_m * 1000.0f;
    QVector3D approachOri_deg = m_calculatedTargetOri_deg;

    qDebug() << "Requesting Pick Sequence: Final Pos(mm):" << finalTargetPos_mm << "Approach Pos(mm):" << approachPos_mm;

    // 3. 메인 윈도우에 시퀀스 실행 요청
    emit requestRobotPickAndReturn(finalTargetPos_mm, finalTargetOri_deg, approachPos_mm, approachOri_deg);
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

        // ✨ [개선] 2D 픽셀 좌표와 3D 포인트 인덱스를 매핑하는 조회 테이블 생성
        const rs2::texture_coordinate* tex_coords = points.get_texture_coordinates();
        m_uv_to_point_idx.assign(IMAGE_WIDTH * IMAGE_HEIGHT, -1);
        for (size_t i = 0; i < points.size(); ++i) {
            int u = static_cast<int>(tex_coords[i].u * IMAGE_WIDTH + 0.5f);
            int v = static_cast<int>(tex_coords[i].v * IMAGE_HEIGHT + 0.5f);
            if (u >= 0 && u < IMAGE_WIDTH && v >= 0 && v < IMAGE_HEIGHT) {
                m_uv_to_point_idx[v * IMAGE_WIDTH + u] = i;
            }
        }

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
        qDebug() << "[WARN] Capture failed. Is camera running?";
        return;
    }
    qDebug() << "[INFO] Capture button pressed. Sending frame to Python.";

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
    }
}


void RealSenseWidget::onDenoisingToggled() {
    m_isDenoisingOn = !m_isDenoisingOn;
    qDebug() << "[INFO] Denoising toggled:" << (m_isDenoisingOn ? "ON" : "OFF");
    updateFrame();
}

void RealSenseWidget::onZFilterToggled() {
    m_pointCloudWidget->m_isZFiltered = !m_pointCloudWidget->m_isZFiltered;
    qDebug() << "[INFO] Z-axis filter toggled:" << (m_pointCloudWidget->m_isZFiltered ? "ON" : "OFF");
    m_pointCloudWidget->processPoints();
}

void RealSenseWidget::runDbscanClustering()
{
    qDebug() << "[INFO] Starting DBSCAN clustering on non-floor points...";

    const rs2::points& points = m_pointCloudWidget->m_points;
    if (!points) {
        qDebug() << "[WARN] No point cloud data to cluster.";
        return;
    }

    std::vector<Point3D> pointsToCluster;
    const rs2::vertex* vertices = points.get_vertices();
    for (size_t i = 0; i < points.size(); ++i) {
        if (i < m_pointCloudWidget->m_floorPoints.size() && !m_pointCloudWidget->m_floorPoints[i] && vertices[i].z > 0) {
            pointsToCluster.push_back({vertices[i].x, vertices[i].y, vertices[i].z, 0, (int)i});
        }
    }

    if (pointsToCluster.empty()) {
        qDebug() << "[INFO] No non-floor points found to cluster.";
        m_clusterIds.assign(points.size(), 0);
        m_pointCloudWidget->processPoints(m_clusterIds);
        return;
    }
    qDebug() << "[INFO] Clustering" << pointsToCluster.size() << "points.";

    float eps = 0.02f;
    int minPts = 10;
    DBSCAN dbscan(eps, minPts, pointsToCluster);
    dbscan.run();

    m_clusterIds.assign(points.size(), 0);
    int clusterCount = 0;
    for (const auto& p : pointsToCluster) {
        m_clusterIds[p.originalIndex] = p.clusterId;
        if (p.clusterId > clusterCount) clusterCount = p.clusterId;
    }
    qDebug() << "[INFO] DBSCAN found" << clusterCount << "clusters.";

    m_pointCloudWidget->processPoints(m_clusterIds);
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
        qDebug() << "[INFO] Floor removal: No dominant floor plane found.";
        m_pointCloudWidget->m_isFloorFiltered = false;
    }
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
