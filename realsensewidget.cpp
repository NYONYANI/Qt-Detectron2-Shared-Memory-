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
#include <QElapsedTimer>
#include <QApplication>
#include <QMatrix3x3> // ✨ [추가] Euler 각도 변환 위해
#include <QQuaternion> // ✨ [추가] Euler 각도 변환 위해
#include <utility> // std::swap 사용

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

    // TCP to Camera 변환 행렬 설정 (로봇 모델에 맞게 조정 필요)
    const float r[] = { 0.0f,  1.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f };
    QMatrix4x4 rotationMatrix(r);
    QMatrix4x4 translationMatrix;
    translationMatrix.translate(-0.080, 0.0325, 0.0308); // 예시 값
    m_tcpToCameraTransform = translationMatrix * rotationMatrix;
}

// ✨ 소멸자 정의
PointCloudWidget::~PointCloudWidget() {}

void PointCloudWidget::setTransforms(const QMatrix4x4& baseToTcp, const QMatrix4x4& tcpToCam)
{
    m_baseToTcpTransform = baseToTcp;
    m_tcpToCameraTransform = tcpToCam;
    update(); // 변환이 업데이트되면 다시 그리도록 요청
}

void PointCloudWidget::drawAxes(float length, float lineWidth)
{
    glLineWidth(lineWidth);
    glBegin(GL_LINES);
    // X축 (빨강)
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(length, 0.0f, 0.0f);
    // Y축 (초록)
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, length, 0.0f);
    // Z축 (파랑)
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, length);
    glEnd();
    glLineWidth(1.0f); // 기본 선 두께로 복원
}

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

// realsensewidget.cpp

void PointCloudWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // --- 뷰 설정 ---
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float aspect = (float)width() / (height() > 0 ? height() : 1);
    float view_size = m_distance * 0.5f; // 거리에 따라 보이는 영역 크기 조절
    glOrtho(-view_size * aspect, view_size * aspect, -view_size, view_size, -100.0, 100.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // 시점 이동 (Pan)
    glTranslatef(m_panX, m_panY, 0.0f);
    // 시점 회전 (Pitch, Yaw)
    glRotatef(m_pitch, 1.0f, 0.0f, 0.0f); // X축 기준 회전
    glRotatef(m_yaw, 0.0f, 1.0f, 0.0f);   // Y축 기준 회전


    // --- 1. 월드 및 3D 데이터 그리기 (깊이 테스트 활성화 상태) ---
    drawGrid(2.0f, 20); // 2x2 크기, 20칸짜리 그리드
    drawAxes(0.2f);     // 월드 좌표계 축 (크기 0.2)

    // --- 로봇 및 포인트 클라우드 그리기 ---
    glPushMatrix(); // 현재 변환 상태 저장

    // 로봇 베이스 -> TCP 변환 적용
    glMultMatrixf(m_baseToTcpTransform.constData());
    drawAxes(0.1f); // 로봇 TCP 좌표계 축 (크기 0.1)
    drawGripper();  // 그리퍼 모델 그리기

    // TCP -> 카메라 변환 적용
    glMultMatrixf(m_tcpToCameraTransform.constData());

    // 포인트 클라우드 그리기
    if (!m_vertexData.empty()) {
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_COLOR_ARRAY);

        glVertexPointer(3, GL_FLOAT, 6 * sizeof(float), m_vertexData.data());
        glColorPointer(3, GL_FLOAT, 6 * sizeof(float), (char*)m_vertexData.data() + 3*sizeof(float));

        glDrawArrays(GL_POINTS, 0, m_vertexData.size() / 6);

        glDisableClientState(GL_COLOR_ARRAY);
        glDisableClientState(GL_VERTEX_ARRAY);
    }

    glPopMatrix(); // 이전 변환 상태 복원

    // --- 2. 3D 오버레이 그리기 (깊이 테스트 비활성화) ---
    glDisable(GL_DEPTH_TEST); // ✨ [핵심] 깊이 테스트 비활성화

    // (기존) 파지점/좌표계 그리기
    drawGraspingSpheres();
    drawTargetPose();
    drawTargetPose_Y_Aligned();
    drawViewPose();

    // (디버깅) 핸들 중심선 및 테스트용 구 그리기
    drawHandleCenterline(); // ✨ 이제 깊이 테스트 없이 그려집니다.

    glEnable(GL_DEPTH_TEST); // ✨ [핵심] 깊이 테스트 다시 활성화
}

// realsensewidget.cpp

// ✨ [수정] 디버깅 구체는 모두 제거. 데이터가 있을 때만 점과 선을 그림.
void PointCloudWidget::drawHandleCenterline()
{
    // 데이터가 없으면(버튼 누르기 전) 아무것도 그리지 않고 즉시 리턴
    if (m_handleCenterlinePoints.isEmpty()) {
        return;
    }

    // --- 1. [디버깅] 곡선의 첫 번째 포인트 위치에 매우 큰 빨간색 점 그리기 ---
    const QVector3D& firstPoint = m_handleCenterlinePoints[0];

    // (데이터 좌표 확인용 로그 - 필요시 주석 해제)
    // qDebug() << "[DEBUG] drawHandleCenterline - First curve point:" << firstPoint;

    glPointSize(10.0f); // 매우 큰 10px 점
    glColor3f(1.0f, 0.0f, 0.0f); // 밝은 빨간색
    glBegin(GL_POINTS);
    glVertex3f(firstPoint.x(), firstPoint.y(), firstPoint.z());
    glEnd();
    glPointSize(1.5f); // 포인트 크기 복원

    // --- 2. [원본] 곡선 그리기 ---
    if (m_handleCenterlinePoints.size() < 2) return; // 점이 2개 이상일 때만 선 그리기

    glLineWidth(8.0f); // 굵은 선
    glColor3f(1.0f, 0.0f, 1.0f); // 마젠타색
    glBegin(GL_LINE_STRIP);
    for (const auto& point : m_handleCenterlinePoints) {
        glVertex3f(point.x(), point.y(), point.z());
    }
    glEnd();
    glLineWidth(1.0f);
}
// ✨ [추가] 3D 핸들 중심선 업데이트 슬롯 구현
void PointCloudWidget::updateHandleCenterline(const QVector<QVector3D> &centerline)
{
    m_handleCenterlinePoints = centerline;
    update(); // 다시 그리도록 요청
}
void PointCloudWidget::initializeGL()
{
    initializeOpenGLFunctions(); // OpenGL 함수 초기화
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f); // 배경색 흰색
    glEnable(GL_DEPTH_TEST); // 깊이 테스트 활성화
    glEnable(GL_PROGRAM_POINT_SIZE); // 프로그램에서 포인트 크기 설정 가능하도록 함
    glPointSize(1.5f); // 포인트 기본 크기 설정
}

void PointCloudWidget::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h > 0 ? h : 1); // 뷰포트 크기 설정
}

void PointCloudWidget::updatePointCloud(const rs2::points& points, const rs2::video_frame& color, const QImage& maskOverlay) {
    m_points = points;
    m_colorFrame = color;
    m_maskOverlay = maskOverlay;
    processPoints(); // 포인트 데이터 처리 및 정점 버퍼 업데이트
}

// ✨ processPoints 정의
void PointCloudWidget::processPoints(const std::vector<int>& clusterIds) {
    if (!m_points || !m_colorFrame || m_points.size() == 0) {
        m_vertexData.clear();
        QMetaObject::invokeMethod(this, "update", Qt::QueuedConnection); // 비동기 업데이트 요청
        return;
    }

    m_vertexData.clear();
    m_vertexData.reserve(m_points.size() * 6); // 메모리 미리 할당

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
        for (int id : clusterIds) {
            if (id > maxClusterId) maxClusterId = id;
        }
        std::mt19937 gen(12345);
        std::uniform_real_distribution<float> distrib(0.0, 1.0);
        for (int i = 1; i <= maxClusterId; ++i) {
            clusterColors[i] = QVector3D(distrib(gen), distrib(gen), distrib(gen));
        }
        clusterColors[-1] = QVector3D(0.5f, 0.5f, 0.5f);
    }

    for (size_t i = 0; i < m_points.size(); ++i) {
        if (vertices[i].z == 0) continue;

        if (m_isZFiltered) {
            QVector3D p_cam(vertices[i].x, vertices[i].y, vertices[i].z);
            QVector3D p_base = cameraToBaseTransform * p_cam;
            if (p_base.z() <= 0) continue;
        }

        if (m_isFloorFiltered && i < m_floorPoints.size() && m_floorPoints[i]) {
            continue;
        }

        if (!useClusters) {
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
                    m_vertexData.push_back(color_data[color_idx + 2] / 255.0f); // R
                    m_vertexData.push_back(color_data[color_idx + 1] / 255.0f); // G
                    m_vertexData.push_back(color_data[color_idx] / 255.0f);     // B
                }
            }
        }
        else {
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


void PointCloudWidget::mousePressEvent(QMouseEvent *event)
{
    m_lastPos = event->pos();
}

void PointCloudWidget::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->position().x() - m_lastPos.x();
    int dy = event->position().y() - m_lastPos.y();

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

void PointCloudWidget::wheelEvent(QWheelEvent *event)
{
    if (event->angleDelta().y() > 0) {
        m_distance *= 1.0f / 1.1f;
    } else {
        m_distance *= 1.1f;
    }
    if (m_distance < 0.01f) m_distance = 0.01f;
    if (m_distance > 50.0f) m_distance = 50.0f;

    update();
}

void PointCloudWidget::keyPressEvent(QKeyEvent *event)
{
    switch (event->key()) {
    case Qt::Key_1:
        if (RealSenseWidget* rs = qobject_cast<RealSenseWidget*>(parentWidget())) {
            rs->onToggleMaskedPoints();
        }
        break;
    case Qt::Key_2: emit denoisingToggled(); break;
    case Qt::Key_3: emit zFilterToggled(); break;
    case Qt::Key_4: emit showXYPlotRequested(); break;
    case Qt::Key_5: emit calculateTargetPoseRequested(); break;
    case Qt::Key_M: emit moveRobotToPreGraspPoseRequested(); break;
    case Qt::Key_D: emit pickAndReturnRequested(); break;
    default:
        QOpenGLWidget::keyPressEvent(event);
    }
}

void PointCloudWidget::updateGraspingPoints(const QVector<QVector3D> &points) {
    m_graspingPoints = points;
    update();
}

void PointCloudWidget::drawGraspingSpheres() {
    if (m_graspingPoints.isEmpty()) return;

    GLUquadric* quadric = gluNewQuadric();
    gluQuadricNormals(quadric, GLU_SMOOTH);

    glColor3f(1.0f, 0.0f, 0.0f); // 빨간색

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
    glColor3f(0.0f, 0.0f, 0.0f); // 검은색

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
    glColor3f(0.5f, 0.0f, 0.8f); // 보라색
    drawAxes(0.1f, 4.0f);
    glPopAttrib();

    glPopMatrix();
}

void PointCloudWidget::drawTargetPose_Y_Aligned()
{
    if (!m_showTargetPose_Y_Aligned) return;

    glPushMatrix();
    glMultMatrixf(m_targetTcpTransform_Y_Aligned.constData());

    glPushAttrib(GL_CURRENT_BIT);
    glColor3f(0.0f, 1.0f, 1.0f); // 청록색
    drawAxes(0.1f, 4.0f);
    glPopAttrib();

    glPopMatrix();
}

void PointCloudWidget::drawViewPose()
{
    if (!m_showViewPose) return;

    glPushMatrix();
    glMultMatrixf(m_viewPoseTransform.constData());

    glPushAttrib(GL_CURRENT_BIT);
    glColor3f(1.0f, 1.0f, 0.0f); // 노란색
    drawAxes(0.1f, 4.0f);
    glPopAttrib();

    glPopMatrix();
}


void PointCloudWidget::updateTargetPoses(const QMatrix4x4 &pose, bool show,
                                         const QMatrix4x4 &pose_y_aligned, bool show_y_aligned,
                                         const QMatrix4x4 &view_pose, bool show_view_pose)
{
    m_targetTcpTransform = pose;
    m_showTargetPose = show;
    m_targetTcpTransform_Y_Aligned = pose_y_aligned;
    m_showTargetPose_Y_Aligned = show_y_aligned;
    m_viewPoseTransform = view_pose;
    m_showViewPose = show_view_pose;
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
    m_depth_to_disparity(true), m_disparity_to_depth(false),
    m_hasCalculatedViewPose(false), // ✨ [추가] 플래그 초기화
    m_hasPCAData(false) // ✨ [추가] 플래그 초기화
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
    connect(m_pointCloudWidget, &PointCloudWidget::pickAndReturnRequested, this, &RealSenseWidget::onPickAndReturnRequested);
    m_layout->addWidget(m_colorLabel, 1);
    m_layout->addWidget(m_pointCloudWidget, 1);
    setLayout(m_layout); // ✨ [오류 수정] setLayout(this) -> setLayout(m_layout)

    m_baseToTcpTransform.setToIdentity();
    const float r[] = { 0.0f,  1.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f };
    QMatrix4x4 rotationMatrix(r);
    QMatrix4x4 translationMatrix;
    translationMatrix.translate(-0.080, 0.0325, 0.0308);
    m_tcpToCameraTransform = translationMatrix * rotationMatrix;

    m_handlePlotWidget = new HandlePlotWidget();
    m_handlePlotWidget->setWindowTitle("Handle Shape Projection (PCA)");
    connect(this, &RealSenseWidget::requestHandleCenterlineUpdate,
            m_pointCloudWidget, &PointCloudWidget::updateHandleCenterline);
    m_pointCloudWidget->setFocus();
}

RealSenseWidget::~RealSenseWidget()
{
    if (m_timer && m_timer->isActive()) m_timer->stop();
    if (data_control) { static_cast<char*>(data_control)[OFFSET_SHUTDOWN] = 1; }
    qDeleteAll(m_plotWidgets);
    m_plotWidgets.clear();
    delete m_handlePlotWidget;
    try { m_pipeline.stop(); } catch (...) {}
}

void RealSenseWidget::setShowPlot(bool show) { m_showPlotWindow = show; }
void RealSenseWidget::onRobotTransformUpdated(const QMatrix4x4 &transform) { m_baseToTcpTransform = transform; }

void RealSenseWidget::onToggleMaskedPoints()
{
    if (m_pointCloudWidget->m_maskOverlay.isNull()) {
        qDebug() << "[WARN] Key '1' (Mask toggle) pressed, but no mask data available."; return;
    }
    m_pointCloudWidget->m_showOnlyMaskedPoints = !m_pointCloudWidget->m_showOnlyMaskedPoints;
    qDebug() << "[INFO] Show only masked points toggled to:" << m_pointCloudWidget->m_showOnlyMaskedPoints;
    m_pointCloudWidget->processPoints();
}

void RealSenseWidget::calculatePCA(const QVector<QVector3D>& points, QVector<QPointF>& projectedPoints)
{
    if (points.size() < 3) {
        qDebug() << "[PCA] Not enough points for PCA:" << points.size();
        m_hasPCAData = false; // ✨ [추가] PCA 실패 시 플래그 설정
        return;
    }

    Eigen::MatrixXf eigenPoints(points.size(), 3);
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for (int i = 0; i < points.size(); ++i) {
        eigenPoints(i, 0) = points[i].x();
        eigenPoints(i, 1) = points[i].y();
        eigenPoints(i, 2) = points[i].z();
        mean += eigenPoints.row(i);
    }
    mean /= points.size();

    eigenPoints.rowwise() -= mean.transpose();

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(eigenPoints, Eigen::ComputeThinU | Eigen::ComputeThinV);

    Eigen::Vector3f pc1 = svd.matrixV().col(0);
    Eigen::Vector3f pc2 = svd.matrixV().col(1);
    Eigen::Vector3f pc3 = svd.matrixV().col(2);

    qDebug() << "[PCA] Handle Plane Normal (PC3):" << pc3(0) << "," << pc3(1) << "," << pc3(2);

    // ✨ [추가] PCA 변환 정보 저장
    m_pcaMean = mean;
    m_pcaPC1 = pc1;
    m_pcaPC2 = pc2;
    m_hasPCAData = true;

    projectedPoints.clear();
    projectedPoints.reserve(points.size());
    for (int i = 0; i < points.size(); ++i) {
        float proj1 = eigenPoints.row(i).dot(pc1);
        float proj2 = eigenPoints.row(i).dot(pc2);
        projectedPoints.append(QPointF(proj1, proj2));
    }
}

// realsensewidget.cpp

void RealSenseWidget::onShowHandlePlot()
{
    qDebug() << "[PLOT] Handle Plot requested.";

    if (m_detectionResults.isEmpty() || !m_pointCloudWidget->m_points) {
        qDebug() << "[PLOT] No detection results or point cloud available.";
        return;
    }
    const rs2::points& currentPoints = m_pointCloudWidget->m_points;
    const rs2::vertex* vertices = currentPoints.get_vertices();
    const int width = IMAGE_WIDTH; const int height = IMAGE_HEIGHT;
    QMatrix4x4 camToBaseTransform = m_baseToTcpTransform * m_tcpToCameraTransform;

    QVector<QVector3D> allHandlePoints3D;

    // ... (기존 3D 포인트 수집 로직 ...
    for (const QJsonValue &cupValue : m_detectionResults) {
        QJsonObject cupResult = cupValue.toObject();
        QString part = "handle";
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
                    int point_idx = m_uv_to_point_idx[v_mask * width + u_mask];
                    if (point_idx != -1) {
                        const rs2::vertex& p = vertices[point_idx];
                        if (p.z > 0) {
                            QVector3D p_cam(p.x, p.y, p.z);
                            QVector3D p_base = camToBaseTransform * p_cam;
                            if (m_pointCloudWidget->m_isZFiltered && p_base.z() <= 0) continue;
                            allHandlePoints3D.append(p_base);
                        }
                    }
                }
            }
        }
    }
    // ... (기존 3D 포인트 수집 로직 끝) ...

    if (allHandlePoints3D.isEmpty()) {
        qDebug() << "[PLOT] No 'handle' 3D points found.";
        m_handleCenterline3D.clear();
        emit requestHandleCenterlineUpdate(m_handleCenterline3D);
        return;
    }

    qDebug() << "[PLOT] Found" << allHandlePoints3D.size() << "handle points. Running PCA...";

    QVector<QPointF> projectedPoints;
    calculatePCA(allHandlePoints3D, projectedPoints); // m_hasPCAData 등이 여기서 설정됨

    m_handlePlotWidget->updateData(projectedPoints);
    m_handlePlotWidget->show();
    m_handlePlotWidget->activateWindow();

    // --- ✨ [수정] 2D 중심선을 3D로 역변환 (디버깅 로그 추가) ---
    m_handleCenterline3D.clear();
    if (m_hasPCAData) {
        qDebug() << "[PLOT DEBUG] PCA data is valid. Getting 2D centerline...";
        QVector<QPointF> centerline2D = m_handlePlotWidget->getSmoothedCenterlinePoints();

        qDebug() << "[PLOT DEBUG] Got" << centerline2D.size() << "2D centerline points.";

        if (centerline2D.size() < 2) {
            qDebug() << "[PLOT WARN] No 2D centerline data found from plot.";
        } else {
            // (로그가 너무 많을 수 있으니 첫 번째 점만 확인)
            const QPointF& p2d_first = centerline2D[0];
            Eigen::Vector3f p3d_first_eigen = m_pcaMean + (p2d_first.x() * m_pcaPC1) + (p2d_first.y() * m_pcaPC2);
            qDebug() << "[PLOT DEBUG] PCA Mean (x,y,z):" << m_pcaMean.x() << m_pcaMean.y() << m_pcaMean.z();
            qDebug() << "[PLOT DEBUG] PCA PC1 (x,y,z):" << m_pcaPC1.x() << m_pcaPC1.y() << m_pcaPC1.z();
            qDebug() << "[PLOT DEBUG] PCA PC2 (x,y,z):" << m_pcaPC2.x() << m_pcaPC2.y() << m_pcaPC2.z();
            qDebug() << "[PLOT DEBUG] First 2D pt (x,y):" << p2d_first.x() << p2d_first.y();
            qDebug() << "[PLOT DEBUG] ==> First 3D pt (x,y,z):" << p3d_first_eigen.x() << p3d_first_eigen.y() << p3d_first_eigen.z();

            m_handleCenterline3D.reserve(centerline2D.size());
            for (const QPointF& p2d : centerline2D) {
                Eigen::Vector3f p3d_eigen = m_pcaMean + (p2d.x() * m_pcaPC1) + (p2d.y() * m_pcaPC2);
                m_handleCenterline3D.append(QVector3D(p3d_eigen.x(), p3d_eigen.y(), p3d_eigen.z()));
            }
            qDebug() << "[PLOT] Converted 2D centerline (" << centerline2D.size() << "pts) to 3D centerline (" << m_handleCenterline3D.size() << "pts).";
        }
    } else {
        qDebug() << "[PLOT ERROR] Cannot convert centerline to 3D: m_hasPCAData is false.";
    }

    // PointCloudWidget에 3D 중심선 전송 (비어있더라도 전송하여 갱신)
    qDebug() << "[PLOT DEBUG] Emitting requestHandleCenterlineUpdate with" << m_handleCenterline3D.size() << "points.";
    emit requestHandleCenterlineUpdate(m_handleCenterline3D);
    // --- [수정] 끝 ---
}
bool RealSenseWidget::calculateGraspingPoses(bool showPlot)
{
    qDebug() << "[CALC] Calculating grasping poses... (ShowPlot: " << showPlot << ")";
    QElapsedTimer timer;
    timer.start();

    if (m_detectionResults.isEmpty() || !m_pointCloudWidget->m_points) {
        qDebug() << "[CALC] No data available for calculation.";
        m_pointCloudWidget->updateTargetPoses(QMatrix4x4(), false, QMatrix4x4(), false, QMatrix4x4(), false); // Ensure poses are hidden
        return false;
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
            QPointF circleCenter(circle.centerX, circle.centerY);
            QPointF handleCentroid = circleCenter;

            if (!singleCupHandlePoints3D.isEmpty()){
                double h_sum_x = 0, h_sum_y = 0;
                for(const auto& p3d : singleCupHandlePoints3D) { h_sum_x += p3d.x(); h_sum_y += p3d.y(); }
                handleCentroid = QPointF(h_sum_x/singleCupHandlePoints3D.size(), h_sum_y/singleCupHandlePoints3D.size());
                QLineF fittedLine(circleCenter, handleCentroid);
                QLineF perpLine(0, 0, -fittedLine.dy(), fittedLine.dx());
                QVector3D perpDir(perpLine.dx(), perpLine.dy(), 0);
                perpDir.normalize();
                QVector3D graspPoint1(circleCenter.x() + circle.radius * perpDir.x(), circleCenter.y() + circle.radius * perpDir.y(), grasp_z);
                m_graspingTargets.append({graspPoint1, perpDir, circleCenter, handleCentroid});
                graspingPointsForViz.append(graspPoint1);
                QVector3D graspPoint2(circleCenter.x() - circle.radius * perpDir.x(), circleCenter.y() - circle.radius * perpDir.y(), grasp_z);
                m_graspingTargets.append({graspPoint2, -perpDir, circleCenter, handleCentroid});
                graspingPointsForViz.append(graspPoint2);
            }
        }
        if(showPlot) {
            QVector<PlotData> bodyPlotData, handlePlotData;
            for(const auto& p3d : singleCupBodyPoints3D) bodyPlotData.append({p3d.toPointF(), Qt::green, "B"});
            for(const auto& p3d : singleCupHandlePoints3D) handlePlotData.append({p3d.toPointF(), Qt::blue, "H"});
            detectedBodyPoints.append(bodyPlotData);
            detectedHandlePoints.append(handlePlotData);
        }
    }

    m_pointCloudWidget->updateGraspingPoints(graspingPointsForViz);
    qDebug() << "[CALC] Grasping point calculation finished in" << timer.elapsed() << "ms.";

    if (showPlot && !detectedBodyPoints.isEmpty()) {
        for (int i = detectedBodyPoints.size(); i < m_plotWidgets.size(); ++i) m_plotWidgets[i]->hide();
        for (int i = 0; i < detectedBodyPoints.size(); ++i) {
            if (i >= m_plotWidgets.size()) m_plotWidgets.append(new XYPlotWidget());
            m_plotWidgets[i]->updateData(detectedBodyPoints[i], detectedHandlePoints[i]);
            m_plotWidgets[i]->setWindowTitle(QString("Cup %1 Fitting Result").arg(i + 1));
            m_plotWidgets[i]->show();
            m_plotWidgets[i]->activateWindow();
        }
    } else if (showPlot) {
        qDebug() << "[CALC] No body points found for plotting.";
    }

    if (m_graspingTargets.isEmpty()) {
        qDebug() << "[CALC] No grasping points calculated. Aborting pose calculation.";
        m_pointCloudWidget->updateTargetPoses(QMatrix4x4(), false, QMatrix4x4(), false, QMatrix4x4(), false);
        return false;
    }

    QVector3D currentTcpPos = m_baseToTcpTransform.column(3).toVector3D();
    float minDistance = std::numeric_limits<float>::max();
    GraspingTarget bestTarget;
    bestTarget = m_graspingTargets[0];
    for (const auto& target : m_graspingTargets) {
        float distance = currentTcpPos.distanceToPoint(target.point);
        if (distance < minDistance) {
            minDistance = distance;
            bestTarget = target;
        }
    }

    const float gripper_z_offset = 0.146f;
    m_calculatedTargetPos_m = bestTarget.point + QVector3D(0, 0, gripper_z_offset);
    const QVector3D& N = bestTarget.direction; // Direction perpendicular to handle line on XY plane

    // Calculate Rz (Yaw) to align TCP X-axis with the grasp direction (N)
    float target_rz_rad_for_X = atan2(N.y(), N.x()); // N 벡터(파지 방향)의 각도

    // ✨ [수정] Y축이 N방향(파지 방향)을 향하도록 Rz를 -90도(M_PI/2) 회전
    float target_rz_rad = target_rz_rad_for_X - (M_PI / 2.0f);


    while (target_rz_rad > M_PI) target_rz_rad -= 2 * M_PI;
    while (target_rz_rad <= -M_PI) target_rz_rad += 2 * M_PI; // 등호 <= 추가

    // ----- [✨ 로봇 각도 변환 로직 시작] -----
    // 1. (Rx, Ry, Rz)로 먼저 4x4 행렬 생성
    // (Standard orientation: Rx=0, Ry=180, Rz=calculated)
    QVector3D euler_RxRyRz_deg(0.0f, 179.9f, qRadiansToDegrees(target_rz_rad));

    m_calculatedTargetPose.setToIdentity();
    m_calculatedTargetPose.translate(m_calculatedTargetPos_m);
    m_calculatedTargetPose.rotate(euler_RxRyRz_deg.z(), 0, 0, 1); // Z (Yaw)
    m_calculatedTargetPose.rotate(euler_RxRyRz_deg.y(), 0, 1, 0); // Y (Pitch)
    m_calculatedTargetPose.rotate(euler_RxRyRz_deg.x(), 1, 0, 0); // X (Roll)

    // 2. 4x4 행렬에서 3x3 회전 행렬 추출
    QMatrix3x3 rotMat = m_calculatedTargetPose.toGenericMatrix<3,3>();

    // 3. 3x3 회전 행렬을 "ZYZ" 규약으로 변환
    QVector3D graspOri_deg_ZYZ = rotationMatrixToEulerAngles(rotMat, "ZYZ");

    // 4. "ZYZ" 각도를 로봇 명령용 (A, B, C)로 변환 (Move View 로직과 동일)
    float cmd_A = graspOri_deg_ZYZ.x() + 180.0f; // Z1 + 180
    float cmd_B = -graspOri_deg_ZYZ.y();         // -Y'
    float cmd_C = graspOri_deg_ZYZ.z() + 180.0f; // Z'' + 180

    // 각도 정규화 (-180 ~ 180)
    while (cmd_A > 180.0f) cmd_A -= 360.0f;
    while (cmd_A <= -180.0f) cmd_A += 360.0f; // 등호 <= 추가
    while (cmd_B > 180.0f) cmd_B -= 360.0f;
    while (cmd_B <= -180.0f) cmd_B += 360.0f; // 등호 <= 추가
    while (cmd_C > 180.0f) cmd_C -= 360.0f;
    while (cmd_C <= -180.0f) cmd_C += 360.0f; // 등호 <= 추가

    // 5. 최종 (A, B, C) 값을 m_calculatedTargetOri_deg에 저장
    m_calculatedTargetOri_deg = QVector3D(cmd_A, cmd_B, cmd_C);
    // ----- [✨ 로봇 각도 변환 로직 끝] -----

    qDebug() << "[CALC] Target Grasp Pose Calculated | Pos(m):" << m_calculatedTargetPos_m
             << "| Ori(A,B,C deg):" << m_calculatedTargetOri_deg
             << "| (Debug ZYZ):" << graspOri_deg_ZYZ;

    // Toggle visualization
    bool show = !m_pointCloudWidget->m_showTargetPose;
    m_pointCloudWidget->updateTargetPoses(m_calculatedTargetPose, show, QMatrix4x4(), false, QMatrix4x4(), false);

    return true;
}


void RealSenseWidget::onShowXYPlot()
{
    qDebug() << "[INFO] Key '4' pressed. Calculating grasp points and showing plot...";
    calculateGraspingPoses(true);
}

void RealSenseWidget::onCalculateTargetPose()
{
    qDebug() << "[INFO] Key '5' pressed. Calculating target pose (no plot)...";
    calculateGraspingPoses(false);
}

void RealSenseWidget::runFullAutomatedSequence()
{
    qDebug() << "[SEQ] Starting full automated sequence...";

    qDebug() << "[SEQ] Step 1/8: Toggling masked points (Key '1')";
    onToggleMaskedPoints();
    QApplication::processEvents();

    qDebug() << "[SEQ] Step 2/8: Toggling denoising (Key '2')";
    onDenoisingToggled();
    QApplication::processEvents();

    qDebug() << "[SEQ] Step 3/8: Toggling Z-Filter (Key '3')";
    onZFilterToggled();
    QApplication::processEvents();

    qDebug() << "[SEQ] Step 4-5/8: Calculating poses (Key '4'+'5', no plot)";
    if (!calculateGraspingPoses(false)) {
        qDebug() << "[SEQ] ERROR: Pose calculation failed. Aborting sequence.";
        return;
    }
    qDebug() << "[SEQ] Pose calculation successful.";

    QVector3D preGraspPos_m = m_calculatedTargetPos_m + QVector3D(0, 0, APPROACH_HEIGHT_M);
    QVector3D preGraspPos_mm = preGraspPos_m * 1000.0f;
    QVector3D preGraspOri_deg = m_calculatedTargetOri_deg; // (A, B, C)

    QVector3D graspPos_mm = m_calculatedTargetPos_m * 1000.0f;
    QVector3D graspOri_deg = m_calculatedTargetOri_deg; // (A, B, C)

    const float LIFT_HEIGHT_M = 0.03f;
    QVector3D liftPos_m = m_calculatedTargetPos_m + QVector3D(0, 0, LIFT_HEIGHT_M);
    QVector3D liftPos_mm = liftPos_m * 1000.0f;
    QVector3D liftOri_deg = graspOri_deg; // (A, B, C)

    // --- [✨ 수정] 로봇 베이스 Y축 정렬 회전 계산 (X축 기준, A값 직접 설정 + 각도 제한 고려) ---
    // 1. 현재 파지 자세(m_calculatedTargetPose - 4x4 행렬)에서 X축 벡터 가져오기
    QVector3D grasp_X_axis = m_calculatedTargetPose.column(0).toVector3D().normalized();

    // 2. X축 벡터의 현재 Rz 각도(라디안) 계산
    float rz_rad_for_X = atan2(grasp_X_axis.y(), grasp_X_axis.x());

    // 3. 목표 각도 (+Y축 또는 -Y축 방향의 A값) 정의
    float target_A_option1 = -90.0f; // Target: TCP X along +Y base
    float target_A_option2 = 90.0f;  // Target: TCP X along -Y base

    // 4. 현재 X축 각도와 목표 방향(+Y 또는 -Y) 사이의 각도 차이(회전량) 계산
    float rotation1_rad = (M_PI / 2.0f) - rz_rad_for_X; // Rotation to align X with +Y base
    float rotation2_rad = (-M_PI / 2.0f) - rz_rad_for_X;// Rotation to align X with -Y base

    // 각도 차이를 (-PI, PI] 범위로 정규화
    while (rotation1_rad > M_PI) rotation1_rad -= 2 * M_PI;
    while (rotation1_rad <= -M_PI) rotation1_rad += 2 * M_PI; // 등호 <= 추가
    while (rotation2_rad > M_PI) rotation2_rad -= 2 * M_PI;
    while (rotation2_rad <= -M_PI) rotation2_rad += 2 * M_PI; // 등호 <= 추가

    float rotation1_deg = qRadiansToDegrees(rotation1_rad);
    float rotation2_deg = qRadiansToDegrees(rotation2_rad);

    // 5. 현재 A축 각도 가져오기
    float current_A_deg = graspOri_deg.x();

    // 6. 각 회전 경로 적용 시 예상되는 최종 A값 계산 및 정규화
    float final_A1 = current_A_deg + rotation1_deg;
    float final_A2 = current_A_deg + rotation2_deg;

    while (final_A1 > 180.0f) final_A1 -= 360.0f;
    while (final_A1 <= -180.0f) final_A1 += 360.0f;
    while (final_A2 > 180.0f) final_A2 -= 360.0f;
    while (final_A2 <= -180.0f) final_A2 += 360.0f;

    // 7. 각도 제한 설정 (예: -170 ~ +170)
    const float LIMIT_HIGH = 170.0f;
    const float LIMIT_LOW = -170.0f;

    bool is_A1_valid = (final_A1 > LIMIT_LOW && final_A1 < LIMIT_HIGH);
    bool is_A2_valid = (final_A2 > LIMIT_LOW && final_A2 < LIMIT_HIGH);

    // 8. 최종 회전 방향 결정 (각도 제한 우선 고려)
    float target_A = 0.0f;
    if (is_A1_valid && !is_A2_valid) {
        target_A = target_A_option1; // 경로 1만 유효하면 경로 1 선택
        qDebug() << "[SEQ ROT SEL] Path 1 chosen (Only Path 1 is valid)";
    } else if (!is_A1_valid && is_A2_valid) {
        target_A = target_A_option2; // 경로 2만 유효하면 경로 2 선택
        qDebug() << "[SEQ ROT SEL] Path 2 chosen (Only Path 2 is valid)";
    } else if (is_A1_valid && is_A2_valid) {
        // 둘 다 유효하면 더 작은 회전량 선택
        if (std::abs(rotation1_rad) <= std::abs(rotation2_rad)) {
            target_A = target_A_option1;
            qDebug() << "[SEQ ROT SEL] Path 1 chosen (Both valid, smaller rotation)";
        } else {
            target_A = target_A_option2;
            qDebug() << "[SEQ ROT SEL] Path 2 chosen (Both valid, smaller rotation)";
        }
    } else {
        // 둘 다 유효하지 않으면, 일단 최소 회전 경로 선택하고 경고
        qWarning() << "[SEQ ROT SEL] Warning: Both rotation paths exceed angle limits!";
        if (std::abs(rotation1_rad) <= std::abs(rotation2_rad)) {
            target_A = target_A_option1;
            qDebug() << "[SEQ ROT SEL] Path 1 chosen (Both invalid, fallback to smaller rotation)";
        } else {
            target_A = target_A_option2;
            qDebug() << "[SEQ ROT SEL] Path 2 chosen (Both invalid, fallback to smaller rotation)";
        }
    }

    // 9. 회전된 최종 Orientation (A, B, C) 설정
    QVector3D rotatedOri_deg = graspOri_deg; // 시작은 원래 파지 자세 A, B, C
    rotatedOri_deg.setX(target_A);           // 계산된 목표 A 값으로 설정 (B, C는 유지)
    // --- [✨ 수정 끝] ---

    // 10. 시퀀스 좌표 설정
    QVector3D rotatePos_mm = liftPos_mm; // 높이는 lift와 동일
    QVector3D rotateOri_deg = rotatedOri_deg; // 계산된 최종 (A, B, C)

    QVector3D placePos_mm = graspPos_mm; // 위치는 grasp와 동일 (회전 전 위치)
    QVector3D placeOri_deg = rotatedOri_deg; // 계산된 최종 (A, B, C)

    qDebug() << "[SEQ] Emitting requestFullPickAndPlaceSequence to RobotController.";
    emit requestFullPickAndPlaceSequence(
        preGraspPos_mm, preGraspOri_deg,
        graspPos_mm, graspOri_deg,
        liftPos_mm, liftOri_deg,
        rotatePos_mm, rotateOri_deg,
        placePos_mm, placeOri_deg
        );
}


void RealSenseWidget::onMoveRobotToPreGraspPose()
{
    qDebug() << "[INFO] Key 'M' pressed. Requesting robot move to pre-grasp and gripper open...";
    if (m_calculatedTargetPos_m.isNull() || m_calculatedTargetOri_deg.isNull()) {
        qWarning() << "[WARN] No target pose has been calculated. Press '5' first.";
        return;
    }
    QVector3D preGraspPos_m = m_calculatedTargetPos_m + QVector3D(0, 0, APPROACH_HEIGHT_M);
    QVector3D preGraspPos_mm = preGraspPos_m * 1000.0f;
    QVector3D preGraspOri_deg = m_calculatedTargetOri_deg;
    qDebug() << "Requesting move to Pre-Grasp Pose: Pos(mm):" << preGraspPos_mm << "Ori(deg):" << preGraspOri_deg;
    emit requestGripperAction(0);
    emit requestRobotMove(preGraspPos_mm, preGraspOri_deg);
}

void RealSenseWidget::onPickAndReturnRequested()
{
    qDebug() << "[INFO] Key 'D' pressed. Requesting pick sequence (grasp only - NO return)...";
    if (m_calculatedTargetPos_m.isNull() || m_calculatedTargetOri_deg.isNull()) {
        qWarning() << "[WARN] Target pose not ready. Press '5' first.";
        return;
    }
    QVector3D finalTargetPos_m = m_calculatedTargetPos_m;
    QVector3D finalTargetPos_mm = finalTargetPos_m * 1000.0f;
    QVector3D finalTargetOri_deg = m_calculatedTargetOri_deg;
    QVector3D approachPos_m = m_calculatedTargetPos_m + QVector3D(0, 0, APPROACH_HEIGHT_M);
    QVector3D approachPos_mm = approachPos_m * 1000.0f;
    QVector3D approachOri_deg = m_calculatedTargetOri_deg;
    qDebug() << "Requesting Pick (Grasp Only): Target Pos(mm):" << finalTargetPos_mm << "Ori(deg):" << finalTargetOri_deg;
    emit requestRobotPickAndReturn(finalTargetPos_mm, finalTargetOri_deg, approachPos_mm, approachOri_deg);
}

void RealSenseWidget::onMoveToYAlignedPoseRequested()
{
    qDebug() << "[INFO] 'MoveButton' (or manual call) pressed. Lift -> Rotate -> Place sequence...";
    if (m_calculatedTargetPos_m.isNull() || m_calculatedTargetOri_deg.isNull()) {
        qWarning() << "[WARN] No target pose has been calculated. Press '5' first.";
        return;
    }
    const float LIFT_HEIGHT_M = 0.03f;
    QVector3D graspPos_mm = m_calculatedTargetPos_m * 1000.0f;
    QVector3D graspOri_deg = m_calculatedTargetOri_deg; // (A, B, C)
    QVector3D liftPos_m = m_calculatedTargetPos_m + QVector3D(0, 0, LIFT_HEIGHT_M);
    QVector3D liftPos_mm = liftPos_m * 1000.0f;

    // --- [✨ 수정] 로봇 베이스 Y축 정렬 회전 계산 (X축 기준, A값 직접 설정 + 각도 제한 고려) ---
    QVector3D grasp_X_axis = m_calculatedTargetPose.column(0).toVector3D().normalized();
    float rz_rad_for_X = atan2(grasp_X_axis.y(), grasp_X_axis.x());
    float target_A_option1 = -90.0f;
    float target_A_option2 = 90.0f;
    float rotation1_rad = (M_PI / 2.0f) - rz_rad_for_X;
    float rotation2_rad = (-M_PI / 2.0f) - rz_rad_for_X;
    while (rotation1_rad > M_PI) rotation1_rad -= 2 * M_PI;
    while (rotation1_rad <= -M_PI) rotation1_rad += 2 * M_PI; // 등호 <= 추가
    while (rotation2_rad > M_PI) rotation2_rad -= 2 * M_PI;
    while (rotation2_rad <= -M_PI) rotation2_rad += 2 * M_PI; // 등호 <= 추가
    float rotation1_deg = qRadiansToDegrees(rotation1_rad);
    float rotation2_deg = qRadiansToDegrees(rotation2_rad);
    float current_A_deg = graspOri_deg.x();
    float final_A1 = current_A_deg + rotation1_deg;
    float final_A2 = current_A_deg + rotation2_deg;
    while (final_A1 > 180.0f) final_A1 -= 360.0f;
    while (final_A1 <= -180.0f) final_A1 += 360.0f;
    while (final_A2 > 180.0f) final_A2 -= 360.0f;
    while (final_A2 <= -180.0f) final_A2 += 360.0f;
    const float LIMIT_HIGH = 170.0f;
    const float LIMIT_LOW = -170.0f;
    bool is_A1_valid = (final_A1 > LIMIT_LOW && final_A1 < LIMIT_HIGH);
    bool is_A2_valid = (final_A2 > LIMIT_LOW && final_A2 < LIMIT_HIGH);
    float target_A = 0.0f;
    if (is_A1_valid && !is_A2_valid) { target_A = target_A_option1; }
    else if (!is_A1_valid && is_A2_valid) { target_A = target_A_option2; }
    else if (is_A1_valid && is_A2_valid) { target_A = (std::abs(rotation1_rad) <= std::abs(rotation2_rad)) ? target_A_option1 : target_A_option2; }
    else { qWarning() << "[SEQ ROT SEL] Warning: Both rotation paths exceed angle limits!"; target_A = (std::abs(rotation1_rad) <= std::abs(rotation2_rad)) ? target_A_option1 : target_A_option2; }
    QVector3D rotatedOri_deg = graspOri_deg;
    rotatedOri_deg.setX(target_A);
    // --- [✨ 수정 끝] ---

    QVector3D placePos_mm = graspPos_mm; // 내려놓는 위치는 회전 전 grasp 위치와 동일

    qDebug() << "[SEQUENCE] Lift to:" << liftPos_mm << "| Rotate to:" << rotatedOri_deg << "| Place at:" << placePos_mm;

    // 시그널 전송: liftPos, graspOri -> liftPos, rotatedOri -> placePos, rotatedOri
    emit requestLiftRotatePlaceSequence(liftPos_mm, graspOri_deg, liftPos_mm, rotatedOri_deg, placePos_mm, rotatedOri_deg);
}

// ✨ [수정] 함수 이름을 onCalculateHandleViewPose로 변경
void RealSenseWidget::onCalculateHandleViewPose()
{
    // ✨ [수정] 로그 메시지 변경
    qInfo() << "[VIEW] 'Move View' requested. Calculating Look-At Pose...";

    // Step 1: 파지 좌표계 계산
    if (!calculateGraspingPoses(false)) {
        qWarning() << "[VIEW] Grasp Pose calculation failed. Aborting move.";
        m_hasCalculatedViewPose = false; // ✨ 계산 실패 시 플래그 초기화
        return;
    }

    // Step 2: 파지 좌표계 유효성 확인
    if (m_calculatedTargetPose.isIdentity()) {
        qWarning() << "[VIEW] Grasp pose data is invalid after calculation. Aborting.";
        m_hasCalculatedViewPose = false; // ✨ 계산 실패 시 플래그 초기화
        return;
    }

    // Step 3: 파지 포인트 위치 추출
    if (m_graspingTargets.isEmpty()) {
        qWarning() << "[VIEW] No grasping targets available. Cannot create look-at view.";
        m_hasCalculatedViewPose = false; // ✨ 계산 실패 시 플래그 초기화
        return;
    }
    QVector3D graspPoint = m_graspingTargets[0].point;

    // Step 4: 뷰 포지션 계산
    QMatrix4x4 viewPoseMatrix = m_calculatedTargetPose;
    const float VIEW_OFFSET_Y_M = 0.3f; // 파지 좌표계 기준 X축 방향으로 이동 (-0.3m)
    viewPoseMatrix.translate(0.0f,VIEW_OFFSET_Y_M, 0.0f);
    const float VIEW_OFFSET_Z_M = -0.0f; // 파지 좌표계 기준 Z축 방향으로 이동 (-0.1m)
    viewPoseMatrix.translate(0.0f, 0.0f, VIEW_OFFSET_Z_M); // ✨ 추가된 Z축 위치 오프셋 적용
    QVector3D viewPos = viewPoseMatrix.column(3).toVector3D(); // 최종 뷰 위치

    // Step 5: 파지 자세의 X축 방향 추출 (새로운 Y축 계산에 사용)
    QVector3D originalX = m_calculatedTargetPose.column(0).toVector3D().normalized();

    // Step 6: 목표 Z축 계산 (파지점보다 살짝 아래를 보도록)
    const float Z_OFFSET_BELOW_GRASP = 0.1f; // Z축으로 2cm 아래 지점을 바라보도록 설정
    QVector3D lookAtTarget = graspPoint - QVector3D(0, 0, Z_OFFSET_BELOW_GRASP); // 목표 지점 Z값 조정
    QVector3D desiredZ = (lookAtTarget - viewPos).normalized(); // 최종 뷰 위치에서 조정된 목표 지점을 향하는 Z벡터

    // Step 7: 새로운 X, Y축 계산
    QVector3D newY = QVector3D::crossProduct(desiredZ, originalX).normalized();
    QVector3D newX = QVector3D::crossProduct(newY, desiredZ).normalized();

    // Step 8: LookAt 행렬 구성 및 Z축 180도 회전 적용
    QMatrix4x4 lookAtMatrix;
    lookAtMatrix.setColumn(0, QVector4D(newX, 0));
    lookAtMatrix.setColumn(1, QVector4D(newY, 0));
    lookAtMatrix.setColumn(2, QVector4D(desiredZ, 0));
    lookAtMatrix.setColumn(3, QVector4D(viewPos, 1));
    const float RELATIVE_RZ_DEG = 90.0f;
    lookAtMatrix.rotate(RELATIVE_RZ_DEG, 0, 0, 1); // 로컬 Z축 기준 회전

    // Step 9: 시각화 업데이트
    m_pointCloudWidget->updateTargetPoses(
        m_calculatedTargetPose, m_pointCloudWidget->m_showTargetPose,
        QMatrix4x4(), false,
        lookAtMatrix, true // 계산된 lookAtMatrix 시각화
        );

    // Step 10: 계산된 값 저장 (이동 명령 위해)
    QVector3D viewOri_deg_ZYX = extractEulerAngles(lookAtMatrix); // 디버깅용 ZYX
    QVector3D viewPos_m = lookAtMatrix.column(3).toVector3D();   // ✨ lookAtMatrix에서 직접 위치 추출
    m_calculatedViewPos_mm = viewPos_m * 1000.0f;
    m_calculatedViewMatrix = lookAtMatrix; // ✨ 계산된 최종 행렬 저장
    m_hasCalculatedViewPose = true;        // ✨ 계산 성공 플래그 설정

    // 디버깅 메시지 출력 (계산 결과 확인용)
    qInfo() << "[VIEW] 1. Calculated Look-At Pose (m / ZYX deg):" // ZYX 기준 로그
            << "Pos:" << viewPos_m
            << "Rot:" << viewOri_deg_ZYX;
    QVector3D current_robot_pos_m = m_baseToTcpTransform.column(3).toVector3D();
    QVector3D current_robot_ori_deg = extractEulerAngles(m_baseToTcpTransform);
    qInfo() << "[VIEW] 2. Current Robot Pose (m / ZYX deg):" // ZYX 기준 로그
            << "Pos:" << current_robot_pos_m
            << "Rot:" << current_robot_ori_deg;
    qInfo() << "[VIEW] Calculated pose is ready for movement (Press MovepointButton).";

}

// ✨ [수정] onMoveToCalculatedHandleViewPose 함수 수정
void RealSenseWidget::onMoveToCalculatedHandleViewPose()
{
    qInfo() << "[VIEW] 'MovepointButton' pressed. Executing move.";
    if (!m_hasCalculatedViewPose) {
        qWarning() << "[VIEW] Move failed: No view pose has been calculated. Press 'Move View' first.";
        return;
    }

    // 1. 저장된 회전 행렬로부터 ZYZ 오일러 각도 계산
    // QMatrix4x4에서 3x3 회전 부분만 추출
    QMatrix3x3 rotMat = m_calculatedViewMatrix.toGenericMatrix<3,3>();
    // ZYZ 오일러 각도 (Z1, Y', Z'') 계산
    QVector3D viewOri_deg_ZYZ = rotationMatrixToEulerAngles(rotMat, "ZYZ");

    // 2. ZYZ 각도를 로봇 명령용 A, B, C로 변환 (추정된 관계식 적용)
    float cmd_A = viewOri_deg_ZYZ.x() + 180.0f; // Z1 + 180
    float cmd_B = -viewOri_deg_ZYZ.y();         // -Y'
    float cmd_C = viewOri_deg_ZYZ.z() + 180.0f; // Z'' + 180

    // 각도 범위를 -180 ~ 180 으로 조정 (±360 주기성 활용)
    // 참고: 로봇 컨트롤러가 이 범위를 자동으로 처리할 수도 있음
    while (cmd_A > 180.0f) cmd_A -= 360.0f;
    while (cmd_A <= -180.0f) cmd_A += 360.0f;
    while (cmd_B > 180.0f) cmd_B -= 360.0f;
    while (cmd_B <= -180.0f) cmd_B += 360.0f;
    while (cmd_C > 180.0f) cmd_C -= 360.0f;
    while (cmd_C <= -180.0f) cmd_C += 360.0f;

    // 로봇에게 전달할 최종 A, B, C 각도
    QVector3D robotCmdOri_deg(cmd_A, cmd_B, cmd_C);

    // 3. 디버깅 메시지 출력 (ZYZ 값과 최종 명령 A,B,C 값 확인)
    qInfo() << "[VIEW] Moving to calculated pose:";
    qInfo() << "  - Pos (mm):" << m_calculatedViewPos_mm;
    // qInfo() << "  - Original Rot (ZYX deg):" << extractEulerAngles(m_calculatedViewMatrix); // ZYX는 이제 참고용
    qInfo() << "  - Calculated Rot (ZYZ deg):" << viewOri_deg_ZYZ; // 계산된 ZYZ 값
    qInfo() << "  - Command Rot (A, B, C deg):" << robotCmdOri_deg; // 최종 변환된 명령 각도

    // 4. 로봇 이동 명령 전송 (변환된 A, B, C 각도 사용)
    emit requestRobotMove(m_calculatedViewPos_mm, robotCmdOri_deg);

    // ✨ 이동 후 계산 플래그 리셋 (선택 사항: 다시 계산해야만 움직이게 하려면 주석 해제)
    // m_hasCalculatedViewPose = false;
}


// 보조 함수: QMatrix4x4에서 Euler 각도 추출 (ZYX 순서) - 기존 함수 유지
QVector3D RealSenseWidget::extractEulerAngles(const QMatrix4x4& matrix)
{
    // R = Rz(yaw) * Ry(pitch) * Rx(roll)
    float m00 = matrix(0, 0), m01 = matrix(0, 1), m02 = matrix(0, 2);
    float m10 = matrix(1, 0), m11 = matrix(1, 1), m12 = matrix(1, 2);
    float m20 = matrix(2, 0), m21 = matrix(2, 1), m22 = matrix(2, 2);

    float pitch = std::asin(-m20);
    float roll, yaw;

    if (std::abs(std::cos(pitch)) > 1e-6) {
        roll = std::atan2(m21, m22);
        yaw = std::atan2(m10, m00);
    } else {
        roll = std::atan2(-m12, m11);
        yaw = 0;
    }

    return QVector3D(
        qRadiansToDegrees(roll),
        qRadiansToDegrees(pitch),
        qRadiansToDegrees(yaw)
        );
}

// ✨ [추가] Euler 각도 변환 함수 (robotcontroller.cpp에서 가져와 수정)
QVector3D RealSenseWidget::rotationMatrixToEulerAngles(const QMatrix3x3& R, const QString& order)
{
    float r11 = R(0,0), r12 = R(0,1), r13 = R(0,2);
    float r21 = R(1,0), r22 = R(1,1), r23 = R(1,2);
    float r31 = R(2,0), r32 = R(2,1), r33 = R(2,2);
    float x=0, y=0, z=0; // 결과 각도를 저장할 변수 (라디안)

    // 참고: qBound는 값을 특정 범위로 제한합니다. asin, acos 입력은 -1 ~ 1 사이여야 함.
    // atan2(y, x)는 아크탄젠트를 계산하며, x와 y의 부호를 고려하여 올바른 사분면의 각도를 반환합니다.

    if (order == "XYZ") { // Roll, Pitch, Yaw 순서 (X먼저 회전)
        y = asin(qBound(-1.0f, r13, 1.0f)); // Pitch
        if (qAbs(qCos(y)) > 1e-6) { // 짐벌락 방지 (Pitch가 +/- 90도일 때)
            x = atan2(-r23, r33); // Roll
            z = atan2(-r12, r11); // Yaw
        } else { // 짐벌락 상태
            x = atan2(r32, r22); // Roll
            z = 0; // Yaw는 Roll과 묶이므로 하나를 0으로 설정
        }
    } else if (order == "ZYX") { // Yaw, Pitch, Roll 순서 (Z먼저 회전, Qt 기본)
        y = asin(-qBound(-1.0f, r31, 1.0f)); // Pitch
        if (qAbs(qCos(y)) > 1e-6) { // 짐벌락 방지
            x = atan2(r32, r33); // Roll
            z = atan2(r21, r11); // Yaw
        } else { // 짐벌락 상태
            // 이 경우 Yaw=0으로 설정하고 Roll 계산 (다른 방법도 가능)
            x = atan2(-r23, r22); // Roll
            z = 0; // Yaw
        }
    } else if (order == "ZYZ") { // Z, Y', Z'' 순서 (로봇 추정 규약)
        y = acos(qBound(-1.0f, r33, 1.0f)); // 두 번째 회전각 (Y')
        if (qAbs(sin(y)) > 1e-6) { // 짐벌락 방지 (Y'가 0 또는 180도일 때)
            x = atan2(r23, r13); // 첫 번째 회전각 (Z1)
            z = atan2(r32, -r31); // 세 번째 회전각 (Z'')
        } else { // 짐벌락 상태
            // Y'가 0 또는 180이면 Z1과 Z''가 같은 축에 대한 회전이 됨
            // Z1+Z'' 또는 Z1-Z'' 만 의미 있고, 개별 값은 결정 불가
            // 여기서는 Z''=0으로 두고 Z1 계산 (관례)
            x = atan2(-r12, r22); // Z1
            z = 0; // Z''
        }
        // ✨ [수정] ZYZ 순서에 맞게 변수 할당: x=Z1, y=Y', z=Z''
        // 위 계산에서 x=Z1, y=Y', z=Z'' 이므로 추가적인 swap 불필요
    }
    // ... (다른 규약 추가 가능: XZY, YXZ, YZX 등) ...
    else {
        qWarning() << "[Angle Debug] Unsupported Euler order:" << order;
        // 지원하지 않는 규약이면 ZYX 결과 반환 (디버깅용)
        QQuaternion quat = QQuaternion::fromRotationMatrix(R);
        QVector3D eulerAngles = quat.toEulerAngles(); // ZYX 순서 (Yaw, Pitch, Roll)
        return eulerAngles;
    }

    // 라디안을 도로 변환하여 반환
    return QVector3D(qRadiansToDegrees(x), qRadiansToDegrees(y), qRadiansToDegrees(z));
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
    qDebug() << "[INFO] Key '2' (Denoising) toggled:" << (m_isDenoisingOn ? "ON" : "OFF");
    updateFrame();
}

void RealSenseWidget::onZFilterToggled() {
    m_pointCloudWidget->m_isZFiltered = !m_pointCloudWidget->m_isZFiltered;
    qDebug() << "[INFO] Key '3' (Z-axis filter) toggled:" << (m_pointCloudWidget->m_isZFiltered ? "ON" : "OFF");
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
