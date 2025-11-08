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
#include <random> // <--- std::mt19937, std::random_device 사용 위해 필요
#include <GL/glu.h>
#include <map>
#include <limits>
#include <QElapsedTimer>
#include <QApplication>
#include <QMatrix3x3>
#include <QQuaternion>
#include <utility> // std::swap
#include <QRandomGenerator> // 랜덤 선택용
#include <algorithm> // ✨ [추가] std::sort 사용
#include <QVBoxLayout> // ✨ [추가] QDialog 레이아웃을 위해 포함

// ✨ [추가] IK Check를 위해 DRFL 전역 변수 선언
#include "DRFLEx.h"
using namespace DRAFramework;
extern CDRFLEx GlobalDrfl;


// ===================================================================
// PointCloudWidget 구현
// ===================================================================

PointCloudWidget::PointCloudWidget(QWidget *parent)
    : QOpenGLWidget(parent), m_colorFrame(nullptr)
{
    setFocusPolicy(Qt::StrongFocus);
    m_yaw = -45.0f; m_pitch = -30.0f; m_panX = 0.0f; m_panY = -0.3f; m_distance = 1.5f;
    m_baseToTcpTransform.setToIdentity();
    const float r[] = { 0.0f, 1.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f };
    QMatrix4x4 rot(r); QMatrix4x4 trans; trans.translate(-0.080, 0.0325, 0.0308);
    m_tcpToCameraTransform = trans * rot;
}

PointCloudWidget::~PointCloudWidget() {}

void PointCloudWidget::setTransforms(const QMatrix4x4& baseToTcp, const QMatrix4x4& tcpToCam)
{ m_baseToTcpTransform = baseToTcp; m_tcpToCameraTransform = tcpToCam; update(); }

void PointCloudWidget::drawAxes(float length, float lineWidth)
{
    glLineWidth(lineWidth);
    glBegin(GL_LINES);
    // X (Red)
    glColor3f(1.0f, 0.0f, 0.0f); glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(length, 0.0f, 0.0f);
    // Y (Green)
    glColor3f(0.0f, 1.0f, 0.0f); glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(0.0f, length, 0.0f);
    // Z (Blue)
    glColor3f(0.0f, 0.0f, 1.0f); glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(0.0f, 0.0f, length);
    glEnd();
    glLineWidth(1.0f);
}
void PointCloudWidget::drawGrid(float size, int divisions)
{
    glLineWidth(1.0f); glColor3f(0.8f, 0.8f, 0.8f);
    float step = size / divisions; float halfSize = size / 2.0f;
    glBegin(GL_LINES);
    for (int i = 0; i <= divisions; ++i) {
        float pos = -halfSize + i * step;
        glVertex3f(-halfSize, pos, 0.0f); glVertex3f(halfSize, pos, 0.0f); // Parallel to X
        glVertex3f(pos, -halfSize, 0.0f); glVertex3f(pos, halfSize, 0.0f); // Parallel to Y
    }
    glEnd();
}

QVector3D PointCloudWidget::setRawBaseFramePoints(const QVector<QVector3D>& points)
{
    makeCurrent(); // OpenGL 컨텍스트 활성화
    m_rawBaseFramePoints.clear();
    m_isRawVizMode = !points.isEmpty();
    m_showRawCentroid = false;
    m_showRawGraspPoint = false;
    m_showRawGraspPose = false;
    m_rawGraspPoint = QVector3D();

    if (points.isEmpty()) {
        doneCurrent();
        update();
        return QVector3D();
    }

    m_rawBaseFramePoints.reserve(points.size() * 6); // 3 pos + 3 color

    // --- 1. 통계 계산 (Pass 1) ---

    // 1a. 돌출도(거리) 계산을 위한 중심점(Centroid) 계산
    QVector3D sum(0.0f, 0.0f, 0.0f);
    for (const auto& p : points) {
        sum += p;
    }
    m_rawCentroid = sum / points.size(); // 멤버 변수에 저장
    m_showRawCentroid = true; // 그리기 플래그 설정


    // 1b. 거리(돌출도) 및 높이(Z)의 최소/최대값 계산
    float minDist = FLT_MAX;
    float maxDist = -FLT_MAX;
    float minZ = FLT_MAX;
    float maxZ = -FLT_MAX;

    QVector<float> distances(points.size());

    for (int i = 0; i < points.size(); ++i) {
        // 거리 계산
        float dist = (points[i] - m_rawCentroid).length();
        distances[i] = dist;
        if (dist < minDist) minDist = dist;
        if (dist > maxDist) maxDist = dist;

        // 높이 계산
        float z = points[i].z();
        if (z < minZ) minZ = z;
        if (z > maxZ) maxZ = z;
    }

    float distRange = maxDist - minDist;
    if (distRange < 1e-6f) distRange = 1.0f; // 0으로 나누기 방지
    float zRange = maxZ - minZ;
    if (zRange < 1e-6f) zRange = 1.0f; // 0으로 나누기 방지

    // --- 2. "곱한 강도" 계산 및 최소/최대값 찾기 (Pass 2) ---
    QVector<float> raw_intensities(points.size());
    float min_raw_intensity = FLT_MAX;
    float max_raw_intensity = -FLT_MAX;
    int bestPointIndex = -1;

    for (int i = 0; i < points.size(); ++i) {
        // 요소 1: 근접도 (0.0=멈, 1.0=가까움)
        float norm_dist = (distances[i] - minDist) / distRange;
        float intensity_prox = 1.0f - norm_dist;

        // 요소 2: 높이 (0.0=낮음, 1.0=높음)
        float norm_height = (points[i].z() - minZ) / zRange;

        // 조합: 두 강도를 "곱함"
        float raw_intensity = intensity_prox * norm_height;

        raw_intensities[i] = raw_intensity;
        if (raw_intensity < min_raw_intensity) min_raw_intensity = raw_intensity;
        if (raw_intensity > max_raw_intensity) {
            max_raw_intensity = raw_intensity;
            bestPointIndex = i;
        }
    }

    // 최고점(파지점) 저장
    if (bestPointIndex != -1) {
        m_rawGraspPoint = points[bestPointIndex];
        m_showRawGraspPoint = true;
    }

    float raw_intensity_range = max_raw_intensity - min_raw_intensity;
    if (raw_intensity_range < 1e-6f) raw_intensity_range = 1.0f; // 0으로 나누기 방지

    // --- 3. 색상 맵 헬퍼 함수 (Lambda) ---
    auto getStandardColor = [](float norm_val) -> QVector3D {
        float r = 0.0f, g = 0.0f, b = 0.0f;
        // 0.0(Blue) -> 0.5(Green) -> 1.0(Red)
        if (norm_val < 0.5f) {
            float t = norm_val * 2.0f; // 0.0 -> 1.0
            r = 0.0f; g = t; b = 1.0f - t;
        } else {
            float t = (norm_val - 0.5f) * 2.0f; // 0.0 -> 1.0
            r = t; g = 1.0f - t; b = 0.0f;
        }
        return QVector3D(std::max(0.0f, std::min(1.0f, r)),
                         std::max(0.0f, std::min(1.0f, g)),
                         std::max(0.0f, std::min(1.0f, b)));
    };

    // --- 4. 정규화 및 최종 색상 적용 (Pass 3) ---
    for (int i = 0; i < points.size(); ++i) {
        const auto& p = points[i];

        // 위치 (X, Y, Z)
        m_rawBaseFramePoints.push_back(p.x());
        m_rawBaseFramePoints.push_back(p.y());
        m_rawBaseFramePoints.push_back(p.z());

        // 4a. "곱한 강도"를 0~1 사이로 정규화
        float final_intensity = (raw_intensities[i] - min_raw_intensity) / raw_intensity_range;

        // 4b. 최종 강도를 (Blue -> Green -> Red) 맵에 매핑
        QVector3D final_color = getStandardColor(final_intensity);

        // 버퍼에 최종 색상 추가
        m_rawBaseFramePoints.push_back(final_color.x());
        m_rawBaseFramePoints.push_back(final_color.y());
        m_rawBaseFramePoints.push_back(final_color.z());
    }

    doneCurrent();
    update();

    return m_rawGraspPoint;
}


void PointCloudWidget::drawRawCentroid()
{
    if (!m_isRawVizMode || !m_showRawCentroid) return;

    GLUquadric* quadric = gluNewQuadric();
    if(quadric) {
        gluQuadricNormals(quadric, GLU_SMOOTH);

        glColor3f(0.0f, 0.0f, 0.0f); // 검은색
        glPushMatrix();
        glTranslatef(m_rawCentroid.x(), m_rawCentroid.y(), m_rawCentroid.z());
        gluSphere(quadric, 0.008, 16, 16); // 8mm
        glPopMatrix();

        gluDeleteQuadric(quadric);
    }
}
void PointCloudWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION); glLoadIdentity();
    float aspect = (float)width() / (height() > 0 ? height() : 1);
    float view_size = m_distance * 0.5f;
    glOrtho(-view_size * aspect, view_size * aspect, -view_size, view_size, -100.0, 100.0);
    glMatrixMode(GL_MODELVIEW); glLoadIdentity();
    glTranslatef(m_panX, m_panY, 0.0f);
    glRotatef(m_pitch, 1.0f, 0.0f, 0.0f); glRotatef(m_yaw, 0.0f, 1.0f, 0.0f);

    // --- 1. 월드 및 3D 데이터 (깊이 테스트 ON) ---
    drawGrid(2.0f, 20);
    drawAxes(0.2f); // 월드 좌표계

    // ✨ [수정] 메인 뷰(!m_isRawVizMode)일 때만 봉을 그립니다.
    if (!m_isRawVizMode) {
        drawPole();
    }

    if (m_isRawVizMode) {
        // Raw Visualization Mode: 월드 좌표계 기준으로 포인트를 그림
        glPointSize(3.0f); // 포인트를 좀 더 크게
        if (!m_rawBaseFramePoints.empty()) {
            glEnableClientState(GL_VERTEX_ARRAY); glEnableClientState(GL_COLOR_ARRAY);
            glVertexPointer(3, GL_FLOAT, 6*sizeof(float), m_rawBaseFramePoints.data());
            glColorPointer(3, GL_FLOAT, 6*sizeof(float), (char*)m_rawBaseFramePoints.data() + 3*sizeof(float));
            glDrawArrays(GL_POINTS, 0, m_rawBaseFramePoints.size() / 6);
            glDisableClientState(GL_COLOR_ARRAY); glDisableClientState(GL_VERTEX_ARRAY);
        }
        glPointSize(1.5f); // 원래 크기로 복원

        // 무게중심 구체를 깊이 테스트가 켜진(ON) 이곳에서 그립니다.
        drawRawCentroid();
        drawRawGraspPoint();
        drawRawGraspPoseAxis();

    } else {
        glPushMatrix(); // 로봇+포인트클라우드 시작
        glMultMatrixf(m_baseToTcpTransform.constData()); // 1. 베이스 -> EF 이동

        // 2. EF 좌표계 그리기 (10cm, 두께 2.0f - drawAxes 기본값)
        drawAxes(0.1f);
        drawGripper();

        // 3. 카메라 좌표계 그리기
        glPushMatrix(); // 현재 (EF) 상태 저장
        glMultMatrixf(m_tcpToCameraTransform.constData()); // 3a. EF -> 카메라 이동
        drawAxes(0.05f, 3.0f); // 3b. 카메라 축 그리기 (5cm, 두께 3.0f)
        glPopMatrix(); // 3c. 다시 EF 좌표계로 복귀

        // 4. 포인트 클라우드를 그리기 위해 EF -> 카메라로 다시 이동
        glMultMatrixf(m_tcpToCameraTransform.constData());
        if (!m_vertexData.empty()) {
            glEnableClientState(GL_VERTEX_ARRAY); glEnableClientState(GL_COLOR_ARRAY);
            glVertexPointer(3, GL_FLOAT, 6*sizeof(float), m_vertexData.data());
            glColorPointer(3, GL_FLOAT, 6*sizeof(float), (char*)m_vertexData.data() + 3*sizeof(float));
            glDrawArrays(GL_POINTS, 0, m_vertexData.size() / 6);
            glDisableClientState(GL_COLOR_ARRAY); glDisableClientState(GL_VERTEX_ARRAY);
        }
        glPopMatrix(); // 로봇+포인트클라우드 끝
    }


    // --- 2. 3D 오버레이 (깊이 테스트 OFF) ---
    glDisable(GL_DEPTH_TEST); // 깊이 테스트 끄기
    if (!m_isRawVizMode) { // Raw Viz 모드에서는 오버레이를 그리지 않음
        drawGraspingSpheres();
        drawTargetPose(); // 보라색 (컵 파지)
        drawTargetPose_Y_Aligned();
        drawViewPose(); // 노란색 (뷰포인트)
        drawHandleCenterline(); // 핸들 중심선 그리기
        drawRandomGraspPose(); // 주황색 (손잡이 파지)
    } else {
        // (비어 있음)
    }
    glEnable(GL_DEPTH_TEST); // 깊이 테스트 다시 켜기
}
void PointCloudWidget::setRawGraspPose(const QMatrix4x4& pose, bool show)
{
    m_rawGraspPose = pose;
    m_showRawGraspPose = show;
    update(); // 뷰 업데이트
}
void PointCloudWidget::initializeGL()
{
    initializeOpenGLFunctions(); // OpenGL 함수 초기화 (QOpenGLFunctions 상속 필요)
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f); // 배경색 흰색
    glEnable(GL_DEPTH_TEST); // 깊이 테스트 활성화
    glEnable(GL_PROGRAM_POINT_SIZE); // 프로그램에서 포인트 크기 설정 가능하도록 함
    glPointSize(1.5f); // 포인트 기본 크기 설정
}

void PointCloudWidget::resizeGL(int w, int h)
{ glViewport(0, 0, w, h > 0 ? h : 1); }

void PointCloudWidget::updatePointCloud(const rs2::points& points, const rs2::video_frame& color, const QImage& maskOverlay)
{ m_points = points; m_colorFrame = color; m_maskOverlay = maskOverlay; processPoints(); }

void PointCloudWidget::processPoints(const std::vector<int>& clusterIds)
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
    int width = m_colorFrame.get_width(); int height = m_colorFrame.get_height();
    bool useMask = !m_maskOverlay.isNull(); bool useClusters = !clusterIds.empty();
    QMatrix4x4 cameraToBaseTransform = m_baseToTcpTransform * m_tcpToCameraTransform;

    std::map<int, QVector3D> clusterColors;
    if (useClusters) {
        int maxClusterId = 0;
        for (int id : clusterIds) { if (id > maxClusterId) maxClusterId = id; }
        std::mt19937 gen(12345);
        std::uniform_real_distribution<float> distrib(0.0, 1.0);
        for (int i = 1; i <= maxClusterId; ++i) clusterColors[i] = QVector3D(distrib(gen), distrib(gen), distrib(gen));
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
            bool isMasked = false; QRgb maskPixel = 0;
            if (useMask && m_maskOverlay.valid(u,v)) {
                maskPixel = m_maskOverlay.pixel(u,v);
                if (qAlpha(maskPixel) > 128) isMasked = true;
            }
            if (!m_showOnlyMaskedPoints || isMasked) {
                m_vertexData.push_back(vertices[i].x); m_vertexData.push_back(vertices[i].y); m_vertexData.push_back(vertices[i].z);
                if (isMasked) {
                    m_vertexData.push_back(qRed(maskPixel)/255.0f); m_vertexData.push_back(qGreen(maskPixel)/255.0f); m_vertexData.push_back(qBlue(maskPixel)/255.0f);
                } else {
                    int color_idx = (u + v * width) * 3;
                    m_vertexData.push_back(color_data[color_idx+2]/255.0f); // R
                    m_vertexData.push_back(color_data[color_idx+1]/255.0f); // G
                    m_vertexData.push_back(color_data[color_idx]/255.0f);   // B
                }
            }
        } else {
            if (i < clusterIds.size() && clusterIds[i] != 0) {
                m_vertexData.push_back(vertices[i].x); m_vertexData.push_back(vertices[i].y); m_vertexData.push_back(vertices[i].z);
                QVector3D color = clusterColors[clusterIds[i]];
                m_vertexData.push_back(color.x()); m_vertexData.push_back(color.y()); m_vertexData.push_back(color.z());
            }
        }
    }
    QMetaObject::invokeMethod(this, "update", Qt::QueuedConnection);
}

void PointCloudWidget::mousePressEvent(QMouseEvent *event)
{ m_lastPos = event->pos(); }
void PointCloudWidget::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->position().x() - m_lastPos.x(); int dy = event->position().y() - m_lastPos.y();
    if (event->buttons() & Qt::LeftButton) { m_yaw += dx * 0.5f; m_pitch += dy * 0.5f; }
    else if (event->buttons() & Qt::RightButton) { m_panX += dx * m_distance * 0.001f; m_panY -= dy * m_distance * 0.001f; }
    m_lastPos = event->pos(); update();
}
void PointCloudWidget::wheelEvent(QWheelEvent *event)
{
    if (event->angleDelta().y() > 0) m_distance *= 1.0f / 1.1f; else m_distance *= 1.1f;
    if (m_distance < 0.01f) m_distance = 0.01f; if (m_distance > 50.0f) m_distance = 50.0f;
    update();
}
void PointCloudWidget::keyPressEvent(QKeyEvent *event)
{
    switch (event->key()) {
    case Qt::Key_1: if (RealSenseWidget* rs = qobject_cast<RealSenseWidget*>(parentWidget())) rs->onToggleMaskedPoints(); break;
    case Qt::Key_2: emit denoisingToggled(); break;
    case Qt::Key_3: emit zFilterToggled(); break;
    case Qt::Key_4: emit showXYPlotRequested(); break;
    case Qt::Key_5: emit calculateTargetPoseRequested(); break;
    case Qt::Key_M: emit moveRobotToPreGraspPoseRequested(); break;
    case Qt::Key_D: emit pickAndReturnRequested(); break;
    default: QOpenGLWidget::keyPressEvent(event);
    }
}
void PointCloudWidget::updateGraspingPoints(const QVector<QVector3D> &points)
{ m_graspingPoints = points; update(); }
void PointCloudWidget::drawGraspingSpheres()
{
    if (m_graspingPoints.isEmpty()) return;
    GLUquadric* quadric = gluNewQuadric(); gluQuadricNormals(quadric, GLU_SMOOTH);
    glColor3f(1.0f, 0.0f, 0.0f); // Red
    for (const auto& point : m_graspingPoints) {
        glPushMatrix(); glTranslatef(point.x(), point.y(), point.z());
        gluSphere(quadric, 0.005, 16, 16); glPopMatrix();
    }
    gluDeleteQuadric(quadric);
}
void PointCloudWidget::drawGripper()
{
    const float z_off=0.146f, hw=0.040f, jaw_l=0.05f; glLineWidth(3.0f); glColor3f(0.0f,0.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(-jaw_l/2, hw, z_off); glVertex3f(jaw_l/2, hw, z_off);
    glVertex3f(-jaw_l/2, -hw, z_off); glVertex3f(jaw_l/2, -hw, z_off);
    glEnd(); glLineWidth(1.0f);
}
void PointCloudWidget::drawTargetPose()
{
    if (!m_showTargetPose) return; glPushMatrix(); glMultMatrixf(m_targetTcpTransform.constData());
    glPushAttrib(GL_CURRENT_BIT); glColor3f(0.5f, 0.0f, 0.8f); drawAxes(0.1f, 4.0f); glPopAttrib();
    glPopMatrix();
}
void PointCloudWidget::drawTargetPose_Y_Aligned()
{
    if (!m_showTargetPose_Y_Aligned) return; glPushMatrix(); glMultMatrixf(m_targetTcpTransform_Y_Aligned.constData());
    glPushAttrib(GL_CURRENT_BIT); glColor3f(0.0f, 1.0f, 1.0f); drawAxes(0.1f, 4.0f); glPopAttrib();
    glPopMatrix();
}
void PointCloudWidget::drawViewPose()
{
    if (!m_showViewPose) return; glPushMatrix(); glMultMatrixf(m_viewPoseTransform.constData());
    glPushAttrib(GL_CURRENT_BIT); glColor3f(1.0f, 1.0f, 0.0f); drawAxes(0.1f, 4.0f); glPopAttrib();
    glPopMatrix();
}
void PointCloudWidget::updateTargetPoses(const QMatrix4x4 &pose, bool show,
                                         const QMatrix4x4 &pose_y_aligned, bool show_y_aligned,
                                         const QMatrix4x4 &view_pose, bool show_view_pose)
{
    m_targetTcpTransform = pose; m_showTargetPose = show;
    m_targetTcpTransform_Y_Aligned = pose_y_aligned; m_showTargetPose_Y_Aligned = show_y_aligned;
    m_viewPoseTransform = view_pose; m_showViewPose = show_view_pose;
    update();
}

void PointCloudWidget::updateHandleCenterline(const QVector<QVector3D> &centerline, const QVector<int> &segmentIds)
{
    m_handleCenterlinePoints = centerline;
    m_handleCenterlineSegmentIds = segmentIds;
    update();
}

void PointCloudWidget::drawHandleCenterline()
{
    int n = m_handleCenterlinePoints.size();
    if (n < 2 || m_handleCenterlineSegmentIds.size() != n) {
        return;
    }

    glLineWidth(8.0f);

    for (int i = 0; i < n - 1; ++i)
    {
        int segmentId = m_handleCenterlineSegmentIds[i];
        if (segmentId < 0 || segmentId >= 3) segmentId = 0;

        if (segmentId == 0) glColor3f(0.0f, 1.0f, 0.0f); // Green
        else if (segmentId == 1) glColor3f(0.0f, 0.0f, 1.0f); // Blue
        else glColor3f(1.0f, 0.0f, 0.0f); // Red

        glBegin(GL_LINES);
        glVertex3f(m_handleCenterlinePoints[i].x(), m_handleCenterlinePoints[i].y(), m_handleCenterlinePoints[i].z());
        glVertex3f(m_handleCenterlinePoints[i+1].x(), m_handleCenterlinePoints[i+1].y(), m_handleCenterlinePoints[i+1].z());
        glEnd();
    }

    glLineWidth(1.0f);
}

void PointCloudWidget::updateRandomGraspPose(const QMatrix4x4 &pose, bool show)
{
    m_randomGraspPose = pose;
    m_showRandomGraspPose = show;
    update();
}

void PointCloudWidget::drawRandomGraspPose()
{
    if (!m_showRandomGraspPose) return;
    glPushMatrix();
    glMultMatrixf(m_randomGraspPose.constData());
    glPushAttrib(GL_CURRENT_BIT);
    glColor3f(1.0f, 0.5f, 0.0f); // Orange
    drawAxes(0.08f, 5.0f);
    glPopAttrib();
    glPopMatrix();
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
    m_showPlotWindow(false), m_depth_to_disparity(true), m_disparity_to_depth(false),
    m_hasCalculatedViewPose(false), m_hasPCAData(false),
    m_showRandomGraspPose(false),
    m_newResultAwaitingFrameUpdate(false)
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
    m_layout->addWidget(m_colorLabel, 1); m_layout->addWidget(m_pointCloudWidget, 1);
    setLayout(m_layout);

    m_baseToTcpTransform.setToIdentity();
    const float r[] = { 0.0f, 1.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f };
    QMatrix4x4 rot(r); QMatrix4x4 trans; trans.translate(-0.080, 0.0325, 0.0308);
    m_tcpToCameraTransform = trans * rot;

    m_handlePlotWidget = new HandlePlotWidget();
    m_handlePlotWidget->setWindowTitle("Handle Shape Projection (PCA + Segmentation)");

    connect(this, &RealSenseWidget::requestHandleCenterlineUpdate,
            m_pointCloudWidget, &PointCloudWidget::updateHandleCenterline);
    connect(this, &RealSenseWidget::requestRandomGraspPoseUpdate,
            m_pointCloudWidget, &PointCloudWidget::updateRandomGraspPose);

    m_pointCloudWidget->setFocus();
}

RealSenseWidget::~RealSenseWidget()
{
    if (m_timer && m_timer->isActive()) m_timer->stop();
    if (data_control) { static_cast<char*>(data_control)[OFFSET_SHUTDOWN] = 1; }
    qDeleteAll(m_plotWidgets); m_plotWidgets.clear();
    delete m_handlePlotWidget;
    // ICP 시각화 다이얼로그가 열려있다면 닫고 삭제
    if (m_icpVizDialog) {
        m_icpVizDialog->close();
        // Qt::WA_DeleteOnClose 속성을 사용했으므로 delete m_icpVizDialog는 필요 없음
    }
    try { m_pipeline.stop(); } catch (...) {}
}

void RealSenseWidget::setShowPlot(bool show)
{ m_showPlotWindow = show; }
void RealSenseWidget::onRobotTransformUpdated(const QMatrix4x4 &transform)
{ m_baseToTcpTransform = transform; }
void RealSenseWidget::onToggleMaskedPoints()
{
    if (m_pointCloudWidget->m_maskOverlay.isNull()) return;
    m_pointCloudWidget->m_showOnlyMaskedPoints = !m_pointCloudWidget->m_showOnlyMaskedPoints;
    qDebug() << "[INFO] Show only masked points toggled to:" << m_pointCloudWidget->m_showOnlyMaskedPoints;
    m_pointCloudWidget->processPoints();
}

// ✨ [수정] IK 체크 헬퍼 함수: ikin 성공 시 반환되는 관절 값(posj)을 로그로 출력
// ✨ [수정] IK 체크 헬퍼 함수: sol_space 인자를 0 -> 2로 변경
bool RealSenseWidget::checkPoseReachable(const QVector3D& pos_mm, const QVector3D& ori_deg)
{
    float target_posx[6];
    target_posx[0] = pos_mm.x();
    target_posx[1] = pos_mm.y();
    target_posx[2] = pos_mm.z();
    target_posx[3] = ori_deg.x();
    target_posx[4] = ori_deg.y();
    target_posx[5] = ori_deg.z();

    // ✨ [수정] ikin 함수의 solution space 인자를 '0'에서 '2'로 변경합니다.
    // '0'이 특이점(J3=0)을 반환했으므로, 다른 해 공간(예: 2)을 시도합니다.
    // (참고: 5_minimal_instruction_sample.cpp 예제에서 '2'를 사용했습니다)
    LPROBOT_POSE ik_solution = GlobalDrfl.ikin(target_posx, 2);

    bool isReachable = (ik_solution != nullptr);

    if (!isReachable) {
        qWarning() << "[IK Check] ❌ Pose UNREACHABLE (ikin(sol_space=2) failed):" << pos_mm << ori_deg;
    } else {
        qDebug() << "[IK Check] ✅ Pose REACHABLE (ikin(sol_space=2) success)." << pos_mm << ori_deg;
        qDebug() << "  - Solution (J1-J6):"
                 << ik_solution->_fPosition[0] << ","
                 << ik_solution->_fPosition[1] << ","
                 << ik_solution->_fPosition[2] << ","
                 << ik_solution->_fPosition[3] << ","
                 << ik_solution->_fPosition[4] << ","
                 << ik_solution->_fPosition[5];
        // ✨ J3 값이 0이 아닌지 확인해보세요.
    }

    return isReachable;
}
bool RealSenseWidget::calculatePCA(const QVector<QVector3D>& points,
                                   QVector<QPointF>& projectedPoints,
                                   Eigen::Vector3f& outMean,
                                   Eigen::Vector3f& outPC1,
                                   Eigen::Vector3f& outPC2,
                                   Eigen::Vector3f& outNormal)
{
    if (points.size() < 3) {
        qDebug() << "[PCA] Not enough points for PCA:" << points.size();
        return false;
    }

    Eigen::MatrixXf eigenPoints(points.size(), 3);
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for (int i = 0; i < points.size(); ++i) {
        eigenPoints(i, 0) = points[i].x(); eigenPoints(i, 1) = points[i].y(); eigenPoints(i, 2) = points[i].z();
        mean += eigenPoints.row(i);
    }
    mean /= points.size();
    eigenPoints.rowwise() -= mean.transpose();
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(eigenPoints, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Vector3f pc1 = svd.matrixV().col(0);
    Eigen::Vector3f pc2 = svd.matrixV().col(1);
    Eigen::Vector3f pc3 = svd.matrixV().col(2);
    qDebug() << "[PCA] Handle Plane Normal (PC3):" << pc3(0) << "," << pc3(1) << "," << pc3(2);

    outMean = mean;
    outPC1 = pc1;
    outPC2 = pc2;
    outNormal = pc3;

    projectedPoints.clear(); projectedPoints.reserve(points.size());
    for (int i = 0; i < points.size(); ++i) {
        float proj1 = eigenPoints.row(i).dot(pc1); float proj2 = eigenPoints.row(i).dot(pc2);
        projectedPoints.append(QPointF(proj1, proj2));
    }

    return true;
}


void RealSenseWidget::onShowHandlePlot(bool showWindow)
{
    qDebug() << "[PLOT] Handle Plot requested. (Show Window:" << showWindow << ")"; // ✨ [추가]
    // 1. 상태 초기화
    m_handleCenterline3D.clear();
    m_handleSegmentIds.clear();
    m_randomGraspPose.setToIdentity();
    m_showRandomGraspPose = false;
    m_hasPCAData = false;
    m_selectedHandlePoints3D.clear(); // ✨ [추가] 선택된 핸들 포인트 초기화

    if (m_detectionResults.isEmpty() || !m_pointCloudWidget->m_points) {
        qDebug() << "[PLOT] No detection results or point cloud available.";
        emit requestHandleCenterlineUpdate(m_handleCenterline3D, m_handleSegmentIds);
        emit requestRandomGraspPoseUpdate(m_randomGraspPose, m_showRandomGraspPose);
        return;
    }

    const rs2::points& currentPoints = m_pointCloudWidget->m_points;
    const rs2::vertex* vertices = currentPoints.get_vertices();
    const int width = IMAGE_WIDTH; const int height = IMAGE_HEIGHT;
    QMatrix4x4 camToBaseTransform = m_baseToTcpTransform * m_tcpToCameraTransform;

    QList<HandleAnalysisResult> allHandleResults;
    int cupIdxCounter = 0;

    // 2. 감지된 모든 손잡이에 대해 PCA 및 거리 계산
    for (const QJsonValue &cupValue : m_detectionResults) {
        cupIdxCounter++;
        QJsonObject cupResult = cupValue.toObject(); QString part = "handle";
        if (!cupResult.contains(part) || !cupResult[part].isObject()) continue;

        HandleAnalysisResult currentResult;
        currentResult.cupIndex = cupIdxCounter;
        QVector<QVector3D> currentHandlePoints3D;

        QJsonObject partData = cupResult[part].toObject();
        QJsonArray rle = partData["mask_rle"].toArray(); QJsonArray shape = partData["mask_shape"].toArray(); QJsonArray offset = partData["offset"].toArray();
        int H = shape[0].toInt(); int W = shape[1].toInt(); int ox = offset[0].toInt(); int oy = offset[1].toInt();
        QVector<uchar> mask_buffer(W * H, 0); int idx = 0; uchar val = 0;
        for(const QJsonValue& run_val : rle) { int len = run_val.toInt(); if(idx + len > W * H) len = W * H - idx; if(len > 0) memset(mask_buffer.data() + idx, val, len); idx += len; val = (val == 0 ? 255 : 0); if(idx >= W * H) break; }

        for(int y = 0; y < H; ++y) {
            for(int x = 0; x < W; ++x) {
                if(mask_buffer[y * W + x] == 255) {
                    int u = ox + x; int v = oy + y;
                    if (u < 0 || u >= width || v < 0 || v >= height) continue;
                    int p_idx = m_uv_to_point_idx[v * width + u];
                    if (p_idx != -1) {
                        const rs2::vertex& p = vertices[p_idx];
                        if (p.z > 0) {
                            QVector3D p_cam(p.x, p.y, p.z);
                            QVector3D p_base = camToBaseTransform * p_cam;
                            if (m_pointCloudWidget->m_isZFiltered && p_base.z() <= 0) continue;
                            currentHandlePoints3D.append(p_base);
                        }
                    }
                }
            }
        }

        if (currentHandlePoints3D.isEmpty()) {
            qDebug() << "[PLOT] Cup" << cupIdxCounter << "has no valid 'handle' 3D points.";
            continue;
        }

        qDebug() << "[PLOT] Cup" << cupIdxCounter << "Found" << currentHandlePoints3D.size() << "handle points. Running PCA...";
        currentResult.handlePoints3D = currentHandlePoints3D;

        Eigen::Vector3f mean, pc1, pc2, normal;
        QVector<QPointF> projected;

        if (calculatePCA(currentHandlePoints3D, projected, mean, pc1, pc2, normal)) {
            currentResult.projectedPoints2D = projected;
            currentResult.pcaMean = mean;
            currentResult.pcaPC1 = pc1;
            currentResult.pcaPC2 = pc2;
            currentResult.pcaNormal = normal;
            currentResult.isValid = true;
            currentResult.distanceToRobot = mean.norm();
            qDebug() << "[PLOT] Cup" << cupIdxCounter << "PCA OK. Distance to base:" << currentResult.distanceToRobot;
            allHandleResults.append(currentResult);
        } else {
            qDebug() << "[PLOT] Cup" << cupIdxCounter << "PCA failed.";
        }
    }

    if (allHandleResults.isEmpty()) {
        qDebug() << "[PLOT] No valid handle results found after PCA.";
        emit requestHandleCenterlineUpdate(m_handleCenterline3D, m_handleSegmentIds);
        emit requestRandomGraspPoseUpdate(m_randomGraspPose, m_showRandomGraspPose);
        return;
    }

    // 3. ✨ [추가] 손잡이를 로봇 베이스와 가까운 순서로 정렬
    std::sort(allHandleResults.begin(), allHandleResults.end(), [](const HandleAnalysisResult& a, const HandleAnalysisResult& b) {
        return a.distanceToRobot < b.distanceToRobot;
    });

    // 4. ✨ [수정] 가장 가까운 손잡이부터 순서대로 IK 체크
    bool foundReachablePose = false;
    for (const HandleAnalysisResult& handle : allHandleResults)
    {
        qDebug() << "[PLOT] Trying Cup" << handle.cupIndex << "(Distance:" << handle.distanceToRobot << ")";

        // 4a. 플롯 위젯을 현재 테스트 중인 손잡이 데이터로 업데이트
        m_handlePlotWidget->updateData(handle.projectedPoints2D);

        // 4b. 현재 손잡이의 3D 중심선 계산
        QVector<QPointF> centerline2D = m_handlePlotWidget->getSmoothedCenterlinePoints();
        m_handleSegmentIds = m_handlePlotWidget->getSegmentIds();
        m_handleCenterline3D.clear();

        if (centerline2D.size() < 2) {
            qWarning() << "[PLOT] Not enough centerline points for this handle. Trying next.";
            continue;
        }

        m_handleCenterline3D.reserve(centerline2D.size());
        for (const QPointF& p2d : centerline2D) {
            Eigen::Vector3f p3d_eigen = handle.pcaMean + (p2d.x() * handle.pcaPC1) + (p2d.y() * handle.pcaPC2);
            m_handleCenterline3D.append(QVector3D(p3d_eigen.x(), p3d_eigen.y(), p3d_eigen.z()));
        }

        // 세그먼트 ID 개수 보정
        if(m_handleCenterline3D.size() != m_handleSegmentIds.size()){
            qWarning() << "[PLOT WARN] Mismatch 3D points/Segment IDs. Adjusting IDs.";
            m_handleSegmentIds.clear();
            m_handleSegmentIds.resize(m_handleCenterline3D.size());
            int third = m_handleCenterline3D.size() / 3;
            for(int i=0; i<m_handleCenterline3D.size(); ++i){
                if (i < third) m_handleSegmentIds[i] = 0;
                else if (i < 2 * third) m_handleSegmentIds[i] = 1;
                else m_handleSegmentIds[i] = 2;
            }
        }

        // 4c. 현재 손잡이로 파지 자세 계산
        QMatrix4x4 calculatedPose;
        QVector3D calculatedPos_m;
        QVector3D calculatedOri_deg;
        bool calcSuccess = calculateRandomGraspPoseOnSegment(2, // Target segment ID (Red)
                                                             handle.pcaMean, handle.pcaPC1, handle.pcaPC2, handle.pcaNormal,
                                                             calculatedPose, calculatedPos_m, calculatedOri_deg);

        if (!calcSuccess) {
            qWarning() << "[PLOT] Pose calculation failed for this handle. Trying next.";
            continue;
        }

        // 4d. ✨ [핵심] 계산된 자세가 도달 가능한지 IK 체크
        QVector3D calculatedPos_mm = calculatedPos_m * 1000.0f;
        if (checkPoseReachable(calculatedPos_mm, calculatedOri_deg))
        {
            // 4e. ✨ [성공] 도달 가능한 자세를 찾음!
            qDebug() << "[PLOT] ✅ SUCCESS: Cup" << handle.cupIndex << "pose is reachable. Selecting this handle.";

            // 이 자세를 최종 자세로 확정
            m_randomGraspPose = calculatedPose;
            m_showRandomGraspPose = true;
            // 이 핸들의 PCA 데이터를 멤버 변수에 저장
            m_pcaMean = handle.pcaMean;
            m_pcaPC1 = handle.pcaPC1;
            m_pcaPC2 = handle.pcaPC2;
            m_pcaNormal = handle.pcaNormal;
            m_hasPCAData = true;
            m_selectedHandlePoints3D = handle.handlePoints3D; // ✨ [추가] 선택된 핸들 포인트 저장

            foundReachablePose = true;
            break; // 루프 종료
        }
        else
        {
            // 4f. ✨ [실패] 도달 불가능. 다음 손잡이로 계속
            qWarning() << "[PLOT] ❌ REJECTED: Cup" << handle.cupIndex << "pose is unreachable. Trying next closest handle.";
        }
    } // --- End of handle loop ---

    // 5. 최종 결과 처리
    if (!foundReachablePose) {
        qWarning() << "[PLOT] 🚨 No reachable handle poses found out of" << allHandleResults.size() << "handles.";
        // 모든 멤버 변수를 초기 상태로 클리어
        m_hasPCAData = false;
        m_handleCenterline3D.clear();
        m_handleSegmentIds.clear();
        m_randomGraspPose.setToIdentity();
        m_showRandomGraspPose = false;
        m_handlePlotWidget->updateData({}); // 플롯 클리어
        m_selectedHandlePoints3D.clear(); // ✨ [추가] 실패 시 포인트 클리어
    } else {
        // ✨ [수정]
        // 성공한 경우, showWindow 플래그가 true일 때만 플롯 창을 띄웁니다.
        if (showWindow) {
            qDebug() << "[PLOT] Showing HandlePlotWidget window.";
            m_handlePlotWidget->show();
            m_handlePlotWidget->activateWindow();
        } else {
            qDebug() << "[PLOT] Skipping HandlePlotWidget window (Auto-sequence).";
        }
    }

    // 6. 3D 뷰 업데이트 (성공했으면 자세가 보이고, 실패했으면 아무것도 안 보임)
    emit requestHandleCenterlineUpdate(m_handleCenterline3D, m_handleSegmentIds);
    emit requestRandomGraspPoseUpdate(m_randomGraspPose, m_showRandomGraspPose);
    emit visionTaskComplete();
}

// ✨ [수정] calculateRandomGraspPoseOnSegment: 시그니처 변경, 멤버 변수 대신 out 매개변수 사용
bool RealSenseWidget::calculateRandomGraspPoseOnSegment(int targetSegmentId,
                                                        const Eigen::Vector3f& mean,
                                                        const Eigen::Vector3f& pc1,
                                                        const Eigen::Vector3f& pc2,
                                                        const Eigen::Vector3f& normal,
                                                        QMatrix4x4& outPose,
                                                        QVector3D& outPos_m,
                                                        QVector3D& outOri_deg)
{
    outPose.setToIdentity();

    // 이 함수가 호출되기 전에 m_handleCenterline3D와 m_handleSegmentIds가
    // 호출자에 의해 (onShowHandlePlot에서) 설정되어 있어야 함
    if (m_handleCenterline3D.size() < 2 || m_handleSegmentIds.size() != m_handleCenterline3D.size()) {
        qWarning() << "[RandGrasp] Cannot calculate: Missing centerline/segment info (Set by caller).";
        return false;
    }

    // 1. 목표 세그먼트 인덱스 수집
    QVector<int> targetIndices;
    for (int i = 0; i < m_handleCenterline3D.size(); ++i) {
        if (m_handleSegmentIds[i] == targetSegmentId) {
            targetIndices.append(i);
        }
    }
    if (targetIndices.isEmpty()) {
        qDebug() << "[RandGrasp] No points found for target segment ID:" << targetSegmentId;
        return false;
    }

    // 2. 랜덤 인덱스 선택
    int randomIndex = -1;
    if (targetIndices.size() > 2) {
        int randIdxInList = QRandomGenerator::global()->bounded(1, targetIndices.size() - 1);
        randomIndex = targetIndices[randIdxInList];
    } else {
        randomIndex = targetIndices[0];
    }
    QVector3D selectedPoint = m_handleCenterline3D[randomIndex];

    // 3. 접선(Tangent) 계산
    QVector3D tangent;
    if (randomIndex == 0) tangent = (m_handleCenterline3D[1] - selectedPoint).normalized();
    else if (randomIndex == m_handleCenterline3D.size() - 1) tangent = (selectedPoint - m_handleCenterline3D[randomIndex - 1]).normalized();
    else tangent = ((selectedPoint - m_handleCenterline3D[randomIndex - 1]).normalized() + (m_handleCenterline3D[randomIndex + 1] - selectedPoint).normalized()).normalized();

    // --- 4. 좌표계(Orientation) 축 계산 ---
    QVector3D yAxis = tangent.normalized();
    QVector3D initialXAxis = QVector3D(normal.x(), normal.y(), normal.z()).normalized();
    QVector3D zAxisForPosition = QVector3D::crossProduct(initialXAxis, yAxis).normalized();

    if (zAxisForPosition.z() < 0.0f) {
        qDebug() << "[RandGrasp] Flipping zAxisForPosition (was" << zAxisForPosition << ")";
        zAxisForPosition = -zAxisForPosition;
        qDebug() << "[RandGrasp] Flipped zAxisForPosition (now" << zAxisForPosition << ")";
    }

    QVector3D finalXAxis = QVector3D::crossProduct(yAxis, zAxisForPosition).normalized();
    QVector3D finalZAxisForOrientation = -zAxisForPosition;

    // --- 5. 최종 TCP Pose 계산 ---
    QVector3D tcpPosition = selectedPoint + zAxisForPosition * GRIPPER_Z_OFFSET;

    outPose.setToIdentity();
    outPose.setColumn(0, QVector4D(-finalXAxis, 0.0f));
    outPose.setColumn(1, QVector4D(yAxis, 0.0f));
    outPose.setColumn(2, QVector4D(finalZAxisForOrientation, 0.0f));
    outPose.setColumn(3, QVector4D(tcpPosition, 1.0f));
    outPose.rotate(-90.0f, 0.0f, 0.0f, 1.0f); // 로컬 Z축(0,0,1) 기준 90도 회전

    outPos_m = tcpPosition; // ✨ [추가] out 매개변수에 위치(m) 저장

    // --- 6. 로봇 (A,B,C) 방향 계산 ---
    QMatrix3x3 rotMat = outPose.toGenericMatrix<3,3>();
    QVector3D graspOriZYZ = rotationMatrixToEulerAngles(rotMat, "ZYZ");
    float cmdA = graspOriZYZ.x() + 180.0f;
    float cmdB = -graspOriZYZ.y();
    float cmdC = graspOriZYZ.z() + 180.0f;
    while(cmdA > 180.0f) cmdA -= 360.0f; while(cmdA <= -180.0f) cmdA += 360.0f;
    while(cmdB > 180.0f) cmdB -= 360.0f; while(cmdB <= -180.0f) cmdB += 360.0f;
    while(cmdC > 180.0f) cmdC -= 360.0f; while(cmdC <= -180.0f) cmdC += 360.0f;

    outOri_deg = QVector3D(cmdA, cmdB, cmdC); // ✨ [추가] out 매개변수에 방향(deg) 저장

    qDebug() << "[RandGrasp] Pose calculation complete. Pos(m):" << outPos_m << "Ori(deg):" << outOri_deg;

    return true;
}

bool RealSenseWidget::calculateGraspingPoses(bool showPlot)
{
    qDebug() << "[CALC] Calculating grasping poses... (ShowPlot: " << showPlot << ")";
    QElapsedTimer timer; timer.start();

    if (m_detectionResults.isEmpty() || !m_pointCloudWidget->m_points) {
        qDebug() << "[CALC] No data available.";
        m_pointCloudWidget->updateTargetPoses(QMatrix4x4(), false, QMatrix4x4(), false, QMatrix4x4(), false);
        return false;
    }
    const rs2::points& currentPoints = m_pointCloudWidget->m_points;
    const rs2::vertex* vertices = currentPoints.get_vertices();
    const int width = IMAGE_WIDTH; const int height = IMAGE_HEIGHT;
    QMatrix4x4 camToBaseTransform = m_baseToTcpTransform * m_tcpToCameraTransform;

    m_graspingTargets.clear(); QVector<QVector3D> graspingPointsForViz;
    QList<QVector<PlotData>> detectedBodyPoints, detectedHandlePoints;

    for (const QJsonValue &cupValue : m_detectionResults) {
        QJsonObject cupResult = cupValue.toObject();
        QVector<QVector3D> body3D, handle3D; QStringList parts = {"body", "handle"};
        for (const QString &part : parts) {
            if (!cupResult.contains(part) || !cupResult[part].isObject()) continue;
            QJsonObject partData = cupResult[part].toObject();
            QJsonArray rle=partData["mask_rle"].toArray(); QJsonArray shp=partData["mask_shape"].toArray(); QJsonArray off=partData["offset"].toArray();
            int H=shp[0].toInt(); int W=shp[1].toInt(); int ox=off[0].toInt(); int oy=off[1].toInt();
            QVector<uchar> mask(W*H, 0); int idx=0; uchar val=0;
            for(const QJsonValue& rv : rle){ int len=rv.toInt(); if(idx+len>W*H) len=W*H-idx; if(len>0) memset(mask.data()+idx, val, len); idx+=len; val=(val==0?255:0); if(idx>=W*H) break; }
            for(int y=0; y<H; ++y) for(int x=0; x<W; ++x) if(mask[y*W+x]==255){ int u=ox+x; int v=oy+y; if(u<0||u>=width||v<0||v>=height) continue; int p_idx=m_uv_to_point_idx[v*width+u]; if(p_idx!=-1){ const rs2::vertex& p=vertices[p_idx]; if(p.z>0){ QVector3D pc(p.x,p.y,p.z); QVector3D pb=camToBaseTransform*pc; if(m_pointCloudWidget->m_isZFiltered&&pb.z()<=0) continue; if(part=="body") body3D.append(pb); else if(part=="handle") handle3D.append(pb);}}}
        }
        if(body3D.isEmpty()) continue;
        QVector<QPointF> body2D; float max_z = -std::numeric_limits<float>::infinity();
        for(const auto& p3d : body3D) { body2D.append(p3d.toPointF()); if (p3d.z() > max_z) max_z = p3d.z(); }
        CircleResult circle = CircleFitter::fitCircleLeastSquares(body2D);
        if (circle.radius > 0 && circle.radius < 1.0) {
            float grasp_z = max_z - 0.01f; QPointF center(circle.centerX, circle.centerY); QPointF handleCentroid = center;
            if (!handle3D.isEmpty()){
                double hx=0, hy=0; for(const auto& p3d : handle3D) { hx+=p3d.x(); hy+=p3d.y(); }
                handleCentroid = QPointF(hx/handle3D.size(), hy/handle3D.size());
                QLineF line(center, handleCentroid); QLineF perp(0,0,-line.dy(),line.dx()); QVector3D dir(perp.dx(),perp.dy(),0); dir.normalize();
                QVector3D p1(center.x()+circle.radius*dir.x(), center.y()+circle.radius*dir.y(), grasp_z); m_graspingTargets.append({p1, dir, center, handleCentroid}); graspingPointsForViz.append(p1);
                QVector3D p2(center.x()-circle.radius*dir.x(), center.y()-circle.radius*dir.y(), grasp_z); m_graspingTargets.append({p2, -dir, center, handleCentroid}); graspingPointsForViz.append(p2);
            }
        }
        if(showPlot){ QVector<PlotData> bodyPlot, handlePlot; for(const auto& p:body3D) bodyPlot.append({p.toPointF(), Qt::green, "B"}); for(const auto& p:handle3D) handlePlot.append({p.toPointF(), Qt::blue, "H"}); detectedBodyPoints.append(bodyPlot); detectedHandlePoints.append(handlePlot); }
    }

    m_pointCloudWidget->updateGraspingPoints(graspingPointsForViz);
    qDebug() << "[CALC] Grasping point calculation finished in" << timer.elapsed() << "ms.";

    if (showPlot && !detectedBodyPoints.isEmpty()) {
        for (int i = detectedBodyPoints.size(); i < m_plotWidgets.size(); ++i) m_plotWidgets[i]->hide();
        for (int i = 0; i < detectedBodyPoints.size(); ++i) {
            if (i >= m_plotWidgets.size()) m_plotWidgets.append(new XYPlotWidget());
            m_plotWidgets[i]->updateData(detectedBodyPoints[i], detectedHandlePoints[i]);
            m_plotWidgets[i]->setWindowTitle(QString("Cup %1 Fitting Result").arg(i + 1));
            m_plotWidgets[i]->show(); m_plotWidgets[i]->activateWindow();
        }
    } else if (showPlot) qDebug() << "[CALC] No body points for plotting.";

    if (m_graspingTargets.isEmpty()) {
        qDebug() << "[CALC] No grasping points calculated.";
        m_pointCloudWidget->updateTargetPoses(QMatrix4x4(), false, QMatrix4x4(), false, QMatrix4x4(), false);
        return false;
    }

    QVector3D currentTcpPos = m_baseToTcpTransform.column(3).toVector3D(); float minDist = std::numeric_limits<float>::max();
    GraspingTarget bestTarget = m_graspingTargets[0];
    for (const auto& target : m_graspingTargets) { float dist = currentTcpPos.distanceToPoint(target.point); if (dist < minDist) { minDist = dist; bestTarget = target; } }

    m_calculatedTargetPos_m = bestTarget.point + QVector3D(0, 0, GRIPPER_Z_OFFSET); // Use constant
    const QVector3D& N = bestTarget.direction;
    float target_rz_rad = atan2(N.y(), N.x()) - (M_PI / 2.0f); // Align Y with N
    while (target_rz_rad > M_PI) target_rz_rad -= 2*M_PI; while (target_rz_rad <= -M_PI) target_rz_rad += 2*M_PI;

    QVector3D euler_RxRyRz_deg(0.0f, 179.9f, qRadiansToDegrees(target_rz_rad));
    m_calculatedTargetPose.setToIdentity(); m_calculatedTargetPose.translate(m_calculatedTargetPos_m);
    m_calculatedTargetPose.rotate(euler_RxRyRz_deg.z(), 0, 0, 1); // Z
    m_calculatedTargetPose.rotate(euler_RxRyRz_deg.y(), 0, 1, 0); // Y
    m_calculatedTargetPose.rotate(euler_RxRyRz_deg.x(), 1, 0, 0); // X
    QMatrix3x3 rotMat = m_calculatedTargetPose.toGenericMatrix<3,3>();
    QVector3D graspOri_deg_ZYZ = rotationMatrixToEulerAngles(rotMat, "ZYZ");
    float cmd_A = graspOri_deg_ZYZ.x() + 180.0f; float cmd_B = -graspOri_deg_ZYZ.y(); float cmd_C = graspOri_deg_ZYZ.z() + 180.0f;
    while(cmd_A>180.0f) cmd_A-=360.0f; while(cmd_A<=-180.0f) cmd_A+=360.0f;
    while(cmd_B>180.0f) cmd_B-=360.0f; while(cmd_B<=-180.0f) cmd_B+=360.0f;
    while(cmd_C>180.0f) cmd_C-=360.0f; while(cmd_C<=-180.0f) cmd_C+=360.0f;
    m_calculatedTargetOri_deg = QVector3D(cmd_A, cmd_B, cmd_C);

    qDebug() << "[CALC] Target Grasp Pose | Pos(m):" << m_calculatedTargetPos_m << "| Ori(A,B,C deg):" << m_calculatedTargetOri_deg;
    bool show = !m_pointCloudWidget->m_showTargetPose;
    m_pointCloudWidget->updateTargetPoses(m_calculatedTargetPose, show, QMatrix4x4(), false, QMatrix4x4(), false);
    return true;
}


void RealSenseWidget::onShowXYPlot()
{ calculateGraspingPoses(true); }
void RealSenseWidget::onCalculateTargetPose()
{ calculateGraspingPoses(false); }

void RealSenseWidget::runFullAutomatedSequence()
{
    qDebug() << "[SEQ] Starting full sequence...";
    onToggleMaskedPoints(); QApplication::processEvents();
    onDenoisingToggled(); QApplication::processEvents();
    onZFilterToggled(); QApplication::processEvents();
    if (!calculateGraspingPoses(false)) { qDebug() << "[SEQ] ERROR: Pose calc failed."; return; }
    qDebug() << "[SEQ] Pose calculation OK.";

    QVector3D preGraspP_m = m_calculatedTargetPos_m + QVector3D(0,0,APPROACH_HEIGHT_M);
    QVector3D preGraspP_mm = preGraspP_m * 1000.0f; QVector3D preGraspO_deg = m_calculatedTargetOri_deg;
    QVector3D graspP_mm = m_calculatedTargetPos_m * 1000.0f; QVector3D graspO_deg = m_calculatedTargetOri_deg;
    const float LIFT_H = 0.03f;
    QVector3D liftP_m = m_calculatedTargetPos_m + QVector3D(0,0,LIFT_H);
    QVector3D liftP_mm = liftP_m * 1000.0f; QVector3D liftO_deg = graspO_deg;

    // Y-axis alignment rotation logic
    QVector3D graspX = m_calculatedTargetPose.column(0).toVector3D().normalized();
    float rzX = atan2(graspX.y(), graspX.x());
    float targetA1=-90.0f, targetA2=90.0f;
    float rot1 = (M_PI/2.0f) - rzX; float rot2 = (-M_PI/2.0f) - rzX;
    while(rot1>M_PI) rot1-=2*M_PI; while(rot1<=-M_PI) rot1+=2*M_PI;
    while(rot2>M_PI) rot2-=2*M_PI; while(rot2<=-M_PI) rot2+=2*M_PI;
    float rot1d=qRadiansToDegrees(rot1), rot2d=qRadiansToDegrees(rot2);
    float currentA = graspO_deg.x(); float finalA1=currentA+rot1d, finalA2=currentA+rot2d;
    while(finalA1>180.0f) finalA1-=360.0f; while(finalA1<=-180.0f) finalA1+=360.0f;
    while(finalA2>180.0f) finalA2-=360.0f; while(finalA2<=-180.0f) finalA2+=360.0f;
    const float LIM_H=170.0f, LIM_L=-170.0f;
    bool v1=(finalA1>LIM_L && finalA1<LIM_H), v2=(finalA2>LIM_L && finalA2<LIM_H);
    float targetA=0.0f;
    if(v1&&!v2) targetA=targetA1; else if(!v1&&v2) targetA=targetA2;
    else if(v1&&v2) targetA = (std::abs(rot1)<=std::abs(rot2)) ? targetA1 : targetA2;
    else { qWarning()<<"[SEQ ROT] Both paths exceed limits!"; targetA = (std::abs(rot1)<=std::abs(rot2)) ? targetA1 : targetA2; }
    QVector3D rotatedO_deg = graspO_deg; rotatedO_deg.setX(targetA);

    QVector3D rotateP_mm = liftP_mm; QVector3D rotateO_deg = rotatedO_deg;
    QVector3D placeP_mm = graspP_mm; QVector3D placeO_deg = rotatedO_deg;

    qDebug() << "[SEQ] Emitting requestFullPickAndPlaceSequence.";
    emit requestFullPickAndPlaceSequence(preGraspP_mm, preGraspO_deg, graspP_mm, graspO_deg,
                                         liftP_mm, liftO_deg, rotateP_mm, rotateO_deg, placeP_mm, placeO_deg);
}


void RealSenseWidget::onMoveRobotToPreGraspPose()
{
    qDebug() << "[INFO] Key 'M' pressed.";
    if (m_calculatedTargetPos_m.isNull() || m_calculatedTargetOri_deg.isNull()) { qWarning() << "[WARN] No target pose. Press '5'."; return; }
    QVector3D preGraspP_m = m_calculatedTargetPos_m + QVector3D(0,0,APPROACH_HEIGHT_M);
    QVector3D preGraspP_mm = preGraspP_m * 1000.0f; QVector3D preGraspO_deg = m_calculatedTargetOri_deg;
    qDebug() << "Req move Pre-Grasp: Pos(mm):" << preGraspP_mm << "Ori(deg):" << preGraspO_deg;
    emit requestGripperAction(0); emit requestRobotMove(preGraspP_mm, preGraspO_deg);
}

void RealSenseWidget::onPickAndReturnRequested()
{
    qDebug() << "[INFO] Key 'D' pressed.";
    if (m_calculatedTargetPos_m.isNull() || m_calculatedTargetOri_deg.isNull()) { qWarning() << "[WARN] Target not ready. Press '5'."; return; }
    QVector3D targetP_mm = m_calculatedTargetPos_m * 1000.0f; QVector3D targetO_deg = m_calculatedTargetOri_deg;
    QVector3D approachP_m = m_calculatedTargetPos_m + QVector3D(0,0,APPROACH_HEIGHT_M);
    QVector3D approachP_mm = approachP_m * 1000.0f; QVector3D approachO_deg = m_calculatedTargetOri_deg;
    qDebug() << "Req Pick (Grasp Only): Target Pos(mm):" << targetP_mm << "Ori(deg):" << targetO_deg;
    emit requestRobotPickAndReturn(targetP_mm, targetO_deg, approachP_mm, approachO_deg);
}

void RealSenseWidget::onMoveToYAlignedPoseRequested()
{
    qDebug() << "[INFO] 'MoveButton' (manual) pressed.";
    if (m_calculatedTargetPos_m.isNull() || m_calculatedTargetOri_deg.isNull()) { qWarning() << "[WARN] No target pose. Press '5'."; return; }
    const float LIFT_H = 0.03f;
    QVector3D graspP_mm = m_calculatedTargetPos_m * 1000.0f; QVector3D graspO_deg = m_calculatedTargetOri_deg;
    QVector3D liftP_m = m_calculatedTargetPos_m + QVector3D(0,0,LIFT_H); QVector3D liftP_mm = liftP_m * 1000.0f;

    // Y-axis alignment rotation logic (same as in runFullAutomatedSequence)
    QVector3D graspX = m_calculatedTargetPose.column(0).toVector3D().normalized(); float rzX = atan2(graspX.y(), graspX.x());
    float tA1=-90.0f, tA2=90.0f; float r1=(M_PI/2.0f)-rzX, r2=(-M_PI/2.0f)-rzX;
    while(r1>M_PI) r1-=2*M_PI; while(r1<=-M_PI) r1+=2*M_PI; while(r2>M_PI) r2-=2*M_PI; while(r2<=-M_PI) r2+=2*M_PI;
    float r1d=qRadiansToDegrees(r1), r2d=qRadiansToDegrees(r2); float cA=graspO_deg.x(); float fA1=cA+r1d, fA2=cA+r2d;
    while(fA1>180.0f) fA1-=360.0f; while(fA1<=-180.0f) fA1+=360.0f; while(fA2>180.0f) fA2-=360.0f; while(fA2<=-180.0f) fA2+=360.0f;
    const float LH=170.0f, LL=-170.0f; bool v1=(fA1>LL && fA1<LH), v2=(fA2>LL && fA2<LH); float tA=0.0f;
    if(v1&&!v2) tA=tA1; else if(!v1&&v2) tA=tA2; else if(v1&&v2) tA=(std::abs(r1)<=std::abs(r2))?tA1:tA2;
    else { qWarning()<<"[SEQ ROT] Both paths exceed limits!"; tA=(std::abs(r1)<=std::abs(r2))?tA1:tA2; }
    QVector3D rotatedO_deg = graspO_deg; rotatedO_deg.setX(tA);

    QVector3D placeP_mm = graspP_mm; // Place at original grasp XY, Z

    qDebug() << "[SEQUENCE] Lift:" << liftP_mm << "| Rotate:" << rotatedO_deg << "| Place:" << placeP_mm;
    emit requestLiftRotatePlaceSequence(liftP_mm, graspO_deg, liftP_mm, rotatedO_deg, placeP_mm, rotatedO_deg);
}

// ✨ [수정] onCalculateHandleViewPose: 위치 계산 로직 변경
void RealSenseWidget::onCalculateHandleViewPose()
{
    qInfo() << "[VIEW] 'Move View' requested. Calculating Look-At Pose...";

    // 1. 파지 자세 및 타겟 리스트 계산
    if (!calculateGraspingPoses(false)) { qWarning() << "[VIEW] Grasp Pose calc failed."; m_hasCalculatedViewPose=false; return; }
    if (m_calculatedTargetPose.isIdentity()) { qWarning() << "[VIEW] Grasp pose invalid."; m_hasCalculatedViewPose=false; return; }
    if (m_graspingTargets.isEmpty()) { qWarning() << "[VIEW] No grasping targets."; m_hasCalculatedViewPose=false; return; }

    // 2. m_calculatedTargetPose를 만든 'bestTarget'을 다시 찾습니다.
    QVector3D graspTargetPos_m = m_calculatedTargetPos_m - QVector3D(0, 0, GRIPPER_Z_OFFSET);
    GraspingTarget* bestTarget = nullptr;
    float minDistToCalculated = std::numeric_limits<float>::max();

    for (GraspingTarget& target : m_graspingTargets) {
        float dist = target.point.distanceToPoint(graspTargetPos_m);
        if (dist < minDistToCalculated) {
            minDistToCalculated = dist;
            bestTarget = &target;
        }
    }

    if (bestTarget == nullptr || minDistToCalculated > 0.001f) {
        qWarning() << "[VIEW] Failed to find the original bestTarget. Using m_graspingTargets[0] as fallback.";
        if(m_graspingTargets.isEmpty()) return; // 방어 코드
        bestTarget = &m_graspingTargets[0];
    } else {
        qInfo() << "[VIEW] Found matching bestTarget for view calculation.";
    }

    // 3. 'bestTarget'의 정보 (베이스 좌표계 기준)
    QPointF bodyCenter2D = bestTarget->circleCenter;
    QPointF handleCentroid2D = bestTarget->handleCentroid;
    QVector3D graspPoint = bestTarget->point;

    // --- 4. 동적 X 오프셋 계산 로직 (EF의 X축 기준) ---
    QVector3D graspX_axis_ef = m_calculatedTargetPose.column(0).toVector3D().normalized();
    QVector3D handlePos3D(handleCentroid2D.x(), handleCentroid2D.y(), graspPoint.z());
    QVector3D vecGraspToHandle = handlePos3D - graspPoint;
    float dot = QVector3D::dotProduct(vecGraspToHandle, graspX_axis_ef);

    const float OFF_X_BASE =0.2f; // 카메라의 X축 (좌우) 오프셋
    const float OFF_Y = -0.05f;      // 카메라의 Y축 (상하) 오프셋
    const float OFF_Z = -0.05f;    // 카메라의 Z축 (전후) 오프셋 (음수 = 뒤로)

    float DYNAMIC_OFF_X;
    if (dot > 0) {
        DYNAMIC_OFF_X = OFF_X_BASE;
        qInfo() << "[VIEW] Handle is on +X_EF side (dot=" << dot << "). Setting CAM_OFF_X to +" << OFF_X_BASE;
    } else {
        DYNAMIC_OFF_X = -OFF_X_BASE;
        qInfo() << "[VIEW] Handle is on -X_EF side (dot=" << dot << "). Setting CAM_OFF_X to -" << OFF_X_BASE;
    }

    // --- 5. (삭제) 카메라의 파지 자세 계산 (QMatrix4x4 cameraGraspPose = ...)

    // --- 6. ✨ [수정] 목표 *EF* 뷰 위치 계산 (파지 좌표계 기준 오프셋) ---
    // m_calculatedTargetPose (파지 좌표계)를 기준으로 로컬 오프셋을 적용합니다.
    // QMatrix4x4::translate()는 로컬 변환(post-multiplication)을 수행합니다.
    QMatrix4x4 required_EF_Pose_for_View = m_calculatedTargetPose;
    required_EF_Pose_for_View.translate(DYNAMIC_OFF_X, OFF_Y, OFF_Z);

    qInfo() << "[VIEW] Grasp EF Pos:     " << m_calculatedTargetPose.column(3).toVector3D();
    qInfo() << "[VIEW] Offsets (X,Y,Z):  " << DYNAMIC_OFF_X << "," << OFF_Y << "," << OFF_Z;
    qInfo() << "[VIEW] New View EF Pos:  " << required_EF_Pose_for_View.column(3).toVector3D();


    // --- 7. ✨ [수정] 목표 *카메라* 뷰 위치 계산 ---
    // 이 새로운 EF 뷰 포즈에 있을 때, 카메라는 어디에 있는지 계산합니다.
    // 이 위치(viewPos_cam)는 LookAt의 시작점이 됩니다.
    QMatrix4x4 cameraViewPose_Matrix = required_EF_Pose_for_View * m_tcpToCameraTransform;
    QVector3D viewPos_cam = cameraViewPose_Matrix.column(3).toVector3D();


    // --- 8. Z 위치 보정 (카메라가 바닥 아래로 가지 않도록) ---
    // (이전 단계의 '7. Z 위치 보정'이 여기로 이동)
    if (viewPos_cam.z() < 0.0f)
    {
        qInfo() << "[VIEW] Original view Z-pos_cam was negative:" << viewPos_cam.z();
        viewPos_cam.setZ(qAbs(viewPos_cam.z()));
        qInfo() << "[VIEW] Mirrored view Z-pos_cam to positive:" << viewPos_cam.z();
    }

    // --- 9. 바라볼 목표 지점(lookTarget) 계산 (베이스 좌표계 기준) ---
    // (이전 단계의 '8. 바라볼 목표 지점'이 여기로 이동)
    const float LOOK_BELOW=0.1f;
    QVector3D lookTarget;
    QPointF graspPoint2D(graspPoint.x(), graspPoint.y());
    float radius = QLineF(bodyCenter2D, graspPoint2D).length();
    QLineF lineBodyToHandle(bodyCenter2D, handleCentroid2D);

    if (radius < 0.001f || lineBodyToHandle.length() < 0.001f) {
        qWarning() << "[VIEW] Radius or handle-body line is too small. Defaulting to graspPoint LookAt.";
        lookTarget = graspPoint - QVector3D(0, 0, LOOK_BELOW);
    } else {
        lineBodyToHandle.setLength(radius);
        QPointF intersectionPoint2D = lineBodyToHandle.p2();
        float lookAtZ = graspPoint.z();
        lookTarget = QVector3D(intersectionPoint2D.x(), intersectionPoint2D.y(), lookAtZ);
        qInfo() << "[VIEW] New LookAt (Intersection on Handle Side):" << lookTarget;
        lookTarget -= QVector3D(0, 0, LOOK_BELOW);
        qInfo() << "[VIEW] Final LookAt (Intersection - Lowered):" << lookTarget;
    }

    // --- 10. 최종 *카메라* 뷰 행렬 계산 (LookAt) ---
    // (이전 단계의 '9. 최종 *카메라* 뷰 행렬'이 여기로 이동)
    QVector3D desiredZ_cam = (lookTarget - viewPos_cam).normalized();

    // 'Up' 벡터를 graspX_axis_ef 대신 월드 Z축으로 고정합니다.
    QVector3D worldUp(0.0f, 0.0f, 1.0f);
    // Z축과 Up 벡터가 거의 평행한 경우 (카메라가 수직으로 위나 아래를 볼 때)
    if (qAbs(QVector3D::dotProduct(desiredZ_cam, worldUp)) > 0.99f) {
        // 'Up' 벡터를 월드 X축으로 대신 사용합니다.
        worldUp = QVector3D(1.0f, 0.0f, 0.0f);
        qInfo() << "[VIEW] LookAt gimbal lock: Using World-X as 'Up'.";
    }

    // newX = Up x Z (OpenGL/Qt는 오른손 좌표계이므로 X = Y x Z 가 아닌 X = Up x Z 로 계산)
    QVector3D newX_cam = QVector3D::crossProduct(worldUp, desiredZ_cam).normalized();
    // newY = Z x X
    QVector3D newY_cam = QVector3D::crossProduct(desiredZ_cam, newX_cam).normalized();

    // ✨ [수정] X축(빨간색) 방향을 "아래쪽" (요청하신 반대 방향)으로 뒤집기 위해
    // X축과 Y축을 모두 반전시킵니다.
    newX_cam = -newX_cam;
    newY_cam = -newY_cam;


    QMatrix4x4 lookAtMat_cam;
    lookAtMat_cam.setColumn(0, QVector4D(newX_cam,0));
    lookAtMat_cam.setColumn(1, QVector4D(newY_cam,0));
    lookAtMat_cam.setColumn(2, QVector4D(desiredZ_cam,0));
    lookAtMat_cam.setColumn(3, QVector4D(viewPos_cam,1)); // <--- 계산된 viewPos_cam (위치) 적용

    // --- 11. 카메라를 이 위치에 두기 위한 *EF* 자세 계산 ---
    // (이전 단계의 '10. 카메라를 ... *EF* 자세 계산'이 여기로 이동)
    QMatrix4x4 required_EF_Pose = lookAtMat_cam * m_tcpToCameraTransform.inverted();

    // --- 12. 계산된 EF 자세를 저장하고 시각화 ---
    // (이전 단계의 '11. 계산된 EF 자세를 저장'이 여기로 이동)
    m_pointCloudWidget->updateTargetPoses(m_calculatedTargetPose, m_pointCloudWidget->m_showTargetPose,
                                          QMatrix4x4(), false,
                                          required_EF_Pose, true); // m_viewPoseTransform에 EF 자세 저장

    QVector3D viewPos_ef = required_EF_Pose.column(3).toVector3D();
    m_calculatedViewPos_mm = viewPos_ef * 1000.0f; // mm 단위 EF 위치
    m_calculatedViewMatrix = required_EF_Pose;     // EF 자세
    m_hasCalculatedViewPose = true;

    qInfo() << "[VIEW] 1. Calc *Camera* Look-At (m): Pos:" << viewPos_cam;
    qInfo() << "[VIEW] 2. Calc *EF* Pose (m): Pos:" << viewPos_ef;
    qInfo() << "[VIEW] 3. Curr Robot (m): Pos:" << m_baseToTcpTransform.column(3).toVector3D();
    qInfo() << "[VIEW] Pose ready for movement (Press MovepointButton).";
    emit visionTaskComplete();
}

void RealSenseWidget::onMoveToCalculatedHandleViewPose()
{
    qInfo() << "[VIEW] 'MovepointButton' pressed.";
    if (!m_hasCalculatedViewPose) {
        qWarning() << "[VIEW] Move failed: No view pose calculated. (Press 'Calc View Point' first)";
        return;
    }

    // --- 1. Get Final Target Pose (already calculated) ---
    // m_calculatedViewMatrix는 목표 EF 자세입니다.
    QMatrix3x3 finalRotMat = m_calculatedViewMatrix.toGenericMatrix<3,3>();
    QVector3D finalOriZYZ = rotationMatrixToEulerAngles(finalRotMat, "ZYZ");
    float f_cmdA = finalOriZYZ.x() + 180.0f; float f_cmdB = -finalOriZYZ.y(); float f_cmdC = finalOriZYZ.z() + 180.0f;
    while(f_cmdA>180.0f) f_cmdA-=360.0f; while(f_cmdA<=-180.0f) f_cmdA+=360.0f;
    while(f_cmdB>180.0f) f_cmdB-=360.0f; while(f_cmdB<=-180.0f) f_cmdB+=360.0f;
    while(f_cmdC>180.0f) f_cmdC-=360.0f; while(f_cmdC<=-180.0f) f_cmdC+=360.0f;

    QVector3D finalCmdOri_deg(f_cmdA, f_cmdB, f_cmdC);
    // m_calculatedViewPos_mm는 목표 EF 위치(mm)입니다.
    QVector3D finalPos_mm = m_calculatedViewPos_mm;


    // --- 2. Get Current Pose (from last update) ---
    // m_baseToTcpTransform은 onRobotTransformUpdated에 의해 계속 업데이트됩니다.
    QVector3D currentPos_m = m_baseToTcpTransform.column(3).toVector3D();
    QVector3D currentPos_mm = currentPos_m * 1000.0f;

    QMatrix3x3 currentRotMat = m_baseToTcpTransform.toGenericMatrix<3,3>();
    QVector3D currentOriZYZ = rotationMatrixToEulerAngles(currentRotMat, "ZYZ");
    float c_cmdA = currentOriZYZ.x() + 180.0f; float c_cmdB = -currentOriZYZ.y(); float c_cmdC = currentOriZYZ.z() + 180.0f;
    while(c_cmdA>180.0f) c_cmdA-=360.0f; while(c_cmdA<=-180.0f) c_cmdA+=360.0f;
    while(c_cmdB>180.0f) c_cmdB-=360.0f; while(c_cmdB<=-180.0f) c_cmdB+=360.0f;
    while(c_cmdC>180.0f) c_cmdC-=360.0f; while(c_cmdC<=-180.0f) c_cmdC+=360.0f;

    QVector3D currentCmdOri_deg(c_cmdA, c_cmdB, c_cmdC);


    // --- 3. Calculate Intermediate Pose (Base X+20cm) ---
    // 현재 위치에서 베이스 X축으로 20cm(200mm) 앞으로 이동
    QVector3D intermediatePos_mm = currentPos_mm + QVector3D(200.0f, 0.0f, 0.0f);
    // 방향은 현재 방향을 그대로 유지
    QVector3D intermediateCmdOri_deg = currentCmdOri_deg;


    // --- 4. Emit Move Commands (Queued) ---
    // requestRobotMove는 RobotController의 moveToPositionAndWait(블로킹)에 연결됩니다.
    // 따라서 로봇 스레드는 1번 명령이 완료될 때까지 대기한 후, 2번 명령을 실행합니다.

    qInfo() << "[VIEW] Queuing 2-step move:";

    qInfo() << "  - 1. (Intermediate) Pos (mm):" << intermediatePos_mm;
    qInfo() << "  - 1. (Intermediate) Ori (deg):" << intermediateCmdOri_deg;
    emit requestRobotMove(intermediatePos_mm, intermediateCmdOri_deg);

    qInfo() << "  - 2. (Final) Pos (mm):" << finalPos_mm;
    qInfo() << "  - 2. (Final) Ori (deg):" << finalCmdOri_deg;
    emit requestRobotMove(finalPos_mm, finalCmdOri_deg);
}

QVector3D RealSenseWidget::extractEulerAngles(const QMatrix4x4& matrix)
{
    float m20 = matrix(2, 0); float pitch = std::asin(-m20); float roll, yaw;
    if (std::abs(std::cos(pitch)) > 1e-6) {
        roll = std::atan2(matrix(2, 1), matrix(2, 2)); yaw = std::atan2(matrix(1, 0), matrix(0, 0));
    } else {
        roll = std::atan2(-matrix(1, 2), matrix(1, 1)); yaw = 0;
    }
    return QVector3D(qRadiansToDegrees(roll), qRadiansToDegrees(pitch), qRadiansToDegrees(yaw));
}

QVector3D RealSenseWidget::rotationMatrixToEulerAngles(const QMatrix3x3& R, const QString& order)
{
    float r11=R(0,0), r12=R(0,1), r13=R(0,2), r21=R(1,0), r22=R(1,1), r23=R(1,2), r31=R(2,0), r32=R(2,1), r33=R(2,2);
    float x=0, y=0, z=0;

    if (order == "XYZ") { y = asin(qBound(-1.0f, r13, 1.0f)); if (qAbs(qCos(y)) > 1e-6) { x = atan2(-r23, r33); z = atan2(-r12, r11); } else { x = atan2(r32, r22); z = 0; } }
    else if (order == "ZYX") { y = asin(-qBound(-1.0f, r31, 1.0f)); if (qAbs(qCos(y)) > 1e-6) { x = atan2(r32, r33); z = atan2(r21, r11); } else { x = atan2(-r23, r22); z = 0; } }
    else if (order == "ZYZ") { y = acos(qBound(-1.0f, r33, 1.0f)); if (qAbs(sin(y)) > 1e-6) { x = atan2(r23, r13); z = atan2(r32, -r31); } else { x = atan2(-r12, r22); z = 0; } }
    else { qWarning() << "[Angle] Unsupported Euler order:" << order; QQuaternion q = QQuaternion::fromRotationMatrix(R); return q.toEulerAngles(); }

    return QVector3D(qRadiansToDegrees(x), qRadiansToDegrees(y), qRadiansToDegrees(z));
}


void RealSenseWidget::updateFrame()
{
    try {
        rs2::frameset frames = m_pipeline.wait_for_frames(1000); if (!frames) return;
        frames = m_align.process(frames);
        rs2::video_frame color = frames.get_color_frame(); rs2::depth_frame depth = frames.get_depth_frame(); if (!color || !depth) return;

        if (m_isDenoisingOn) {
            depth = m_dec_filter.process(depth); depth = m_depth_to_disparity.process(depth);
            depth = m_spat_filter.process(depth); depth = m_disparity_to_depth.process(depth);
        }

        m_pointcloud.map_to(color);
        rs2::points points = m_pointcloud.calculate(depth);
        const rs2::texture_coordinate* tex = points.get_texture_coordinates();
        m_uv_to_point_idx.assign(IMAGE_WIDTH * IMAGE_HEIGHT, -1);
        for (size_t i = 0; i < points.size(); ++i) {
            int u = static_cast<int>(tex[i].u*IMAGE_WIDTH+0.5f); int v = static_cast<int>(tex[i].v*IMAGE_HEIGHT+0.5f);
            if (u>=0 && u<IMAGE_WIDTH && v>=0 && v<IMAGE_HEIGHT) m_uv_to_point_idx[v*IMAGE_WIDTH+u] = i;
        }

        m_pointCloudWidget->setTransforms(m_baseToTcpTransform, m_tcpToCameraTransform);
        QImage maskOverlay(color.get_width(), color.get_height(), QImage::Format_ARGB32_Premultiplied);
        maskOverlay.fill(Qt::transparent);
        if (!m_detectionResults.isEmpty()) drawMaskOverlay(maskOverlay, m_detectionResults);
        m_pointCloudWidget->updatePointCloud(points, color, maskOverlay);

        m_latestFrame = cv::Mat(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP).clone();
        m_currentImage = cvMatToQImage(m_latestFrame);

        if (!m_detectionResults.isEmpty()) drawMaskOverlay(m_currentImage, m_detectionResults);
        if(m_isProcessing) { QPainter p(&m_currentImage); p.setPen(Qt::yellow); p.drawText(20, 30, "Processing..."); }
        m_colorLabel->setPixmap(QPixmap::fromImage(m_currentImage).scaled(m_colorLabel->size(), Qt::KeepAspectRatio));
        m_pointCloudWidget->update();
        if (m_newResultAwaitingFrameUpdate) {
            m_newResultAwaitingFrameUpdate = false; // 플래그 리셋
            qDebug() << "[INFO] Emitting visionTaskComplete() for auto-sequence (AFTER frame update).";
            emit visionTaskComplete(); // 시퀀서의 m_visionWaitLoop->quit() 호출
        }

    } catch (const rs2::error& e) { qWarning() << "RS error:" << e.what(); }
    catch (...) { qWarning() << "Unknown error in updateFrame"; }
}

void RealSenseWidget::startCameraStream()
{ try { m_pipeline.start(m_config); m_timer->start(33); } catch (const rs2::error &e) { qCritical() << "RS Error:" << e.what(); } }
QImage RealSenseWidget::cvMatToQImage(const cv::Mat &mat)
{ if(mat.empty()) return QImage(); return QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_BGR888).rgbSwapped(); }

void RealSenseWidget::captureAndProcess(bool isAutoSequence)
{
    if (m_isProcessing || m_latestFrame.empty()) {
        qDebug() << "[WARN] Capture failed. (Processing:" << m_isProcessing << "FrameEmpty:" << m_latestFrame.empty() << ")";

        // ✨ [수정] 자동 시퀀스 중복 요청이 거부된 경우 (isAutoSequence && m_isProcessing)
        // 여기서 visionTaskComplete()를 emit하면 시퀀스가 즉시 '성공'한 것으로
        // 오해하고 (잘못된 데이터로) 다음 단계를 진행하므로,
        // 시그널을 절대 보내지 않고 경고만 출력 후 리턴해야 합니다.
        if (isAutoSequence && m_isProcessing) {
            qWarning() << "[SEQ] Auto-sequence capture request IGNORED (already processing). Sequencer will wait.";
            // emit visionTaskComplete(); // ✨ [삭제]
        }
        return;
    }

    qDebug() << "[INFO] Sending frame to Python. (AutoSequence:" << isAutoSequence << ")";

    m_isAutoSequenceCapture = isAutoSequence;

    sendImageToPython(m_latestFrame);
    m_isProcessing = true;
    m_processingTimer.start(); // <-- ✨ [추가] 타임아웃 타이머 시작
    m_resultTimer->start(100);
}

void RealSenseWidget::checkProcessingResult()
{
    if (!m_isProcessing) { m_resultTimer->stop(); return; }
    QJsonArray results = receiveResultsFromPython();

    if (!results.isEmpty()) {
        qDebug() << "[INFO] Received" << results.size() << "results.";
        m_detectionResults = results;
        m_isProcessing = false; // <-- 정상적으로 플래그 리셋
        m_resultTimer->stop();  // <-- 정상적으로 타이머 중지

        // ✨ [수정] 자동 시퀀스의 일부로 캡처된 경우
        if (m_isAutoSequenceCapture) {
            // ✨ [수정] 즉시 시그널을 보내지 않고, updateFrame에서 처리하도록 플래그만 설정
            qDebug() << "[INFO] Results received. Flagging for frame update before signaling complete.";
            m_newResultAwaitingFrameUpdate = true;
            // emit visionTaskComplete(); // ✨ [이동] 이 라인을 updateFrame() 끝으로 이동
        } else {
            qDebug() << "[INFO] Manual capture complete. (Not emitting signal)";
        }

        // 플래그 초기화
        m_isAutoSequenceCapture = false;
    } else {
        // ✨ [추가] 타임아웃 체크 로직
        // 결과가 비어있다면 (Python이 아직 응답 안함)
        // 타임아웃을 확인합니다. (예: 5000ms = 5초)
        if (m_processingTimer.elapsed() > 5000) {
            qWarning() << "[WARN] Python detection timed out after 5 seconds.";
            qWarning() << "[WARN] Forcibly resetting processing flag. Python server may need restart.";

            m_isProcessing = false; // <-- ✨ [추가] 강제로 플래그 리셋
            m_resultTimer->stop();  // <-- ✨ [추가] 결과 확인 타이머 중지
            m_isAutoSequenceCapture = false; // <-- ✨ [추가] 시퀀스 플래그도 리셋
        }
        // 타임아웃이 아니라면, 타이머는 계속 실행되며 다음 100ms 후에 다시 체크합니다.
    }
}
void RealSenseWidget::onDenoisingToggled()
{ m_isDenoisingOn = !m_isDenoisingOn; qDebug() << "[INFO] Denoising:" << m_isDenoisingOn; updateFrame(); }
void RealSenseWidget::onZFilterToggled()
{ m_pointCloudWidget->m_isZFiltered = !m_pointCloudWidget->m_isZFiltered; qDebug() << "[INFO] Z-Filter:" << m_pointCloudWidget->m_isZFiltered; m_pointCloudWidget->processPoints(); }

void RealSenseWidget::runDbscanClustering()
{
    qDebug() << "[INFO] Starting DBSCAN...";
    const rs2::points& points = m_pointCloudWidget->m_points; if (!points) { qDebug() << "[WARN] No points to cluster."; return; }
    std::vector<Point3D> pointsToCluster; const rs2::vertex* vertices = points.get_vertices();
    for (size_t i = 0; i < points.size(); ++i) { if (i < m_pointCloudWidget->m_floorPoints.size() && !m_pointCloudWidget->m_floorPoints[i] && vertices[i].z > 0) pointsToCluster.push_back({vertices[i].x, vertices[i].y, vertices[i].z, 0, (int)i}); }
    if (pointsToCluster.empty()) { qDebug() << "[INFO] No non-floor points."; m_clusterIds.assign(points.size(), 0); m_pointCloudWidget->processPoints(m_clusterIds); return; }
    qDebug() << "[INFO] Clustering" << pointsToCluster.size() << "points.";
    float eps = 0.02f; int minPts = 10; DBSCAN dbscan(eps, minPts, pointsToCluster); dbscan.run();
    m_clusterIds.assign(points.size(), 0); int clusterCount = 0;
    for (const auto& p : pointsToCluster) { m_clusterIds[p.originalIndex] = p.clusterId; if (p.clusterId > clusterCount) clusterCount = p.clusterId; }
    qDebug() << "[INFO] DBSCAN found" << clusterCount << "clusters.";
    m_pointCloudWidget->processPoints(m_clusterIds);
}

void RealSenseWidget::findFloorPlaneRANSAC() {
    const rs2::points& points = m_pointCloudWidget->m_points; if (!points || points.size() < 100) { qDebug() << "[INFO] Floor: Not enough points."; m_pointCloudWidget->m_isFloorFiltered = false; return; }
    const rs2::vertex* vertices = points.get_vertices(); const size_t num_points = points.size();
    std::vector<int> best_inliers;
    std::mt19937 gen(std::random_device{}());
    std::uniform_int_distribution<> distrib(0, num_points - 1);
    for (int i = 0; i < 100; ++i) {
        rs2::vertex p1=vertices[distrib(gen)], p2=vertices[distrib(gen)], p3=vertices[distrib(gen)];
        float a=(p2.y-p1.y)*(p3.z-p1.z)-(p2.z-p1.z)*(p3.y-p1.y), b=(p2.z-p1.z)*(p3.x-p1.x)-(p2.x-p1.x)*(p3.z-p1.z), c=(p2.x-p1.x)*(p3.y-p1.y)-(p2.y-p1.y)*(p3.x-p1.x);
        float d=-(a*p1.x+b*p1.y+c*p1.z); float norm=sqrt(a*a+b*b+c*c); if (norm < 1e-6) continue;
        a/=norm; b/=norm; c/=norm; d/=norm; std::vector<int> current_inliers;
        for (size_t j=0; j<num_points; ++j) { const rs2::vertex& p=vertices[j]; if(p.x==0&&p.y==0&&p.z==0) continue; if (std::abs(a*p.x+b*p.y+c*p.z+d) < 0.01) current_inliers.push_back(j); }
        if (current_inliers.size() > best_inliers.size()) best_inliers = current_inliers;
    }
    if (best_inliers.size() > num_points / 4) {
        qDebug() << "[INFO] Floor found:" << best_inliers.size() << "inliers.";
        m_pointCloudWidget->m_floorPoints.assign(num_points, false); for (int idx : best_inliers) m_pointCloudWidget->m_floorPoints[idx] = true;
        m_pointCloudWidget->m_isFloorFiltered = true;
    } else {
        qDebug() << "[INFO] No floor found."; m_pointCloudWidget->m_isFloorFiltered = false;
    }
}

void RealSenseWidget::drawMaskOverlay(QImage &image, const QJsonArray &results) {
    if (image.isNull()) return; QPainter painter(&image); painter.setRenderHint(QPainter::Antialiasing, true); int cupIndex = 1;
    for (const QJsonValue &value : results) {
        QJsonObject cupResult = value.toObject(); QStringList parts = {"body", "handle"};
        for (const QString &part : parts) {
            if (!cupResult.contains(part) || !cupResult[part].isObject()) continue;
            QJsonObject partData = cupResult[part].toObject(); if (!partData.contains("mask_rle") || !partData.contains("center")) continue;
            QJsonArray rle=partData["mask_rle"].toArray(); QJsonArray shp=partData["mask_shape"].toArray(); QJsonArray off=partData["offset"].toArray(); QJsonArray center=partData["center"].toArray();
            int H=shp[0].toInt(); int W=shp[1].toInt(); int ox=off[0].toInt(); int oy=off[1].toInt(); int cX=center[0].toInt(); int cY=center[1].toInt(); int cls=partData["cls_id"].toInt();
            QVector<uchar> mask(W*H, 0); int idx=0; uchar val=0;
            for(const QJsonValue& rv : rle) { int len=rv.toInt(); if(idx+len>W*H) len=W*H-idx; if(len>0) memset(mask.data()+idx, val, len); idx+=len; val=(val==0?255:0); if(idx>=W*H) break; }
            QImage mask_img(mask.constData(), W, H, W, QImage::Format_Alpha8); QColor maskColor = (cls==1) ? QColor(0,255,0,150) : QColor(0,0,255,150);
            QImage colored(W, H, QImage::Format_ARGB32_Premultiplied); colored.fill(maskColor);
            QPainter p(&colored); p.setCompositionMode(QPainter::CompositionMode_DestinationIn); p.drawImage(0, 0, mask_img); p.end();
            painter.drawImage(ox, oy, colored); painter.setPen(Qt::white); painter.setFont(QFont("Arial", 12, QFont::Bold));
            QString label = QString("cup%1: %2").arg(cupIndex).arg(part); painter.drawText(cX, cY, label);
        } cupIndex++;
    }
}

bool RealSenseWidget::initSharedMemory() {
    shm_unlink(SHM_IMAGE_NAME); shm_unlink(SHM_RESULT_NAME); shm_unlink(SHM_CONTROL_NAME);
    sem_unlink(SEM_IMAGE_NAME); sem_unlink(SEM_RESULT_NAME); sem_unlink(SEM_CONTROL_NAME);
    auto setup_shm = [&](const char* name, int size, int& fd, void*& data) {
        fd = shm_open(name, O_RDWR | O_CREAT, 0666);
        if (fd != -1 && ftruncate(fd, size) != -1) { data = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0); if (data != MAP_FAILED) return true; }
        return false;
    };
    if (!setup_shm(SHM_IMAGE_NAME, IMAGE_SIZE, fd_image, data_image) ||
        !setup_shm(SHM_RESULT_NAME, RESULT_SIZE, fd_result, data_result) ||
        !setup_shm(SHM_CONTROL_NAME, CONTROL_SIZE, fd_control, data_control)) return false;
    auto setup_sem = [&](const char* name, sem_t*& sem) { sem = sem_open(name, O_CREAT, 0666, 1); return sem != SEM_FAILED; };
    if (!setup_sem(SEM_IMAGE_NAME, sem_image) || !setup_sem(SEM_RESULT_NAME, sem_result) || !setup_sem(SEM_CONTROL_NAME, sem_control)) return false;
    sem_wait(sem_control); memset(data_control, 0, CONTROL_SIZE); sem_post(sem_control);
    return true;
}

void RealSenseWidget::sendImageToPython(const cv::Mat &mat) {
    if (mat.empty() || !data_image || !sem_image || !data_control || !sem_control) return;
    sem_wait(sem_image); memcpy(data_image, mat.data, mat.total() * mat.elemSize()); sem_post(sem_image);
    sem_wait(sem_control); static_cast<char*>(data_control)[OFFSET_NEW_FRAME] = 1; sem_post(sem_control);
}

QJsonArray RealSenseWidget::receiveResultsFromPython() {
    QJsonArray results; if (!data_result || !sem_result || !data_control || !sem_control) return results;
    sem_wait(sem_control); char flag = static_cast<char*>(data_control)[OFFSET_RESULT_READY]; sem_post(sem_control);
    if (flag == 0) return results;
    sem_wait(sem_result); quint32 size; memcpy(&size, data_result, sizeof(quint32));
    if (size > 0 && (int)size <= RESULT_SIZE - (int)sizeof(quint32)) {
        QByteArray jsonData(static_cast<char*>(data_result) + sizeof(quint32), size);
        QJsonDocument doc = QJsonDocument::fromJson(jsonData); if (doc.isArray()) results = doc.array();
    }
    sem_post(sem_result);
    sem_wait(sem_control); static_cast<char*>(data_control)[OFFSET_RESULT_READY] = 0; sem_post(sem_control);
    return results;
}

void RealSenseWidget::onShowICPVisualization()
{
    qInfo() << "[ICP] 'ICPButton' clicked. Finding closest handle from *current* capture...";

    if (m_detectionResults.isEmpty() || !m_pointCloudWidget->m_points) {
        qWarning() << "[ICP] No detection results or point cloud available. Press 'Capture' first.";
        return;
    }

    const rs2::points& currentPoints = m_pointCloudWidget->m_points;
    const rs2::vertex* vertices = currentPoints.get_vertices();
    const int width = IMAGE_WIDTH; const int height = IMAGE_HEIGHT;
    QMatrix4x4 camToBaseTransform = m_baseToTcpTransform * m_tcpToCameraTransform;

    QList<HandleAnalysisResult> allHandleResults;
    int cupIdxCounter = 0;

    for (const QJsonValue &cupValue : m_detectionResults) {
        cupIdxCounter++;
        QJsonObject cupResult = cupValue.toObject(); QString part = "handle";
        if (!cupResult.contains(part) || !cupResult[part].isObject()) continue;

        HandleAnalysisResult currentResult;
        currentResult.cupIndex = cupIdxCounter;
        QVector<QVector3D> currentHandlePoints3D;

        QJsonObject partData = cupResult[part].toObject();
        QJsonArray rle = partData["mask_rle"].toArray(); QJsonArray shape = partData["mask_shape"].toArray(); QJsonArray offset = partData["offset"].toArray();
        int H = shape[0].toInt(); int W = shape[1].toInt(); int ox = offset[0].toInt(); int oy = offset[1].toInt();
        QVector<uchar> mask_buffer(W * H, 0); int idx = 0; uchar val = 0;
        for(const QJsonValue& run_val : rle) { int len = run_val.toInt(); if(idx + len > W * H) len = W * H - idx; if(len > 0) memset(mask_buffer.data() + idx, val, len); idx += len; val = (val == 0 ? 255 : 0); if(idx >= W * H) break; }

        Eigen::Vector3f mean = Eigen::Vector3f::Zero();
        int pointCount = 0;

        for(int y = 0; y < H; ++y) {
            for(int x = 0; x < W; ++x) {
                if(mask_buffer[y * W + x] == 255) {
                    int u = ox + x; int v = oy + y;
                    if (u < 0 || u >= width || v < 0 || v >= height) continue;
                    int p_idx = m_uv_to_point_idx[v * width + u];
                    if (p_idx != -1) {
                        const rs2::vertex& p = vertices[p_idx];
                        if (p.z > 0) {
                            QVector3D p_cam(p.x, p.y, p.z);
                            QVector3D p_base = camToBaseTransform * p_cam;
                            if (m_pointCloudWidget->m_isZFiltered && p_base.z() <= 0) continue;

                            currentHandlePoints3D.append(p_base);
                            mean += Eigen::Vector3f(p_base.x(), p_base.y(), p_base.z());
                            pointCount++;
                        }
                    }
                }
            }
        }

        if (pointCount > 0) {
            mean /= pointCount;
            currentResult.handlePoints3D = currentHandlePoints3D;
            currentResult.distanceToRobot = mean.norm();
            currentResult.isValid = true;
            allHandleResults.append(currentResult);
        }
    }

    if (allHandleResults.isEmpty()) {
        qWarning() << "[ICP] No valid 'handle' 3D points found in the current capture.";
        return;
    }

    std::sort(allHandleResults.begin(), allHandleResults.end(), [](const HandleAnalysisResult& a, const HandleAnalysisResult& b) {
        return a.distanceToRobot < b.distanceToRobot;
    });

    QVector<QVector3D> focusedHandlePoints = allHandleResults[0].handlePoints3D;
    qInfo() << "[ICP] Found " << allHandleResults.size() << " handles. Focusing on closest one (Cup "
            << allHandleResults[0].cupIndex << ", " << focusedHandlePoints.size() << " points).";


    if (m_icpVizDialog == nullptr) {
        m_icpVizDialog = new QDialog(this);
        m_icpVizDialog->setWindowTitle("Focused Handle Point Cloud (Base Frame)");
        m_icpVizDialog->setAttribute(Qt::WA_DeleteOnClose);
        m_icpVizDialog->resize(640, 480);
        m_icpPointCloudWidget = new PointCloudWidget(m_icpVizDialog);
        m_icpPointCloudWidget->setTransforms(QMatrix4x4(), QMatrix4x4());
        QVBoxLayout* layout = new QVBoxLayout(m_icpVizDialog);
        layout->addWidget(m_icpPointCloudWidget);
        m_icpVizDialog->setLayout(layout);

        connect(m_icpVizDialog, &QDialog::finished, [this](){
            m_icpVizDialog = nullptr;
            m_icpPointCloudWidget = nullptr;
            qDebug() << "[ICP] Visualization dialog closed.";
        });

        connect(this, &RealSenseWidget::requestRawGraspPoseUpdate,
                m_icpPointCloudWidget, &PointCloudWidget::setRawGraspPose);
    }

    // --- 3. PCA 실행 및 파지 좌표계 계산 ---

    Eigen::Vector3f mean_eigen, pc1, pc2, normal_eigen;
    QVector<QPointF> projectedPoints;
    bool pca_ok = calculatePCA(focusedHandlePoints, projectedPoints, mean_eigen, pc1, pc2, normal_eigen);

    QVector3D graspPoint = m_icpPointCloudWidget->setRawBaseFramePoints(focusedHandlePoints);
    QVector3D centroid(mean_eigen.x(), mean_eigen.y(), mean_eigen.z()); // PCA 평균(무게중심)

    if (!graspPoint.isNull() && pca_ok)
    {
        // 1. Z축 계산 (파지점 -> 무게중심) [높은 우선순위]
        QVector3D Z_axis = (centroid - graspPoint).normalized();

        // 2. Y축 "힌트" (PCA의 '짧은 방향' Normal) [낮은 우선순위]
        QVector3D Y_axis_hint = QVector3D(normal_eigen.x(), normal_eigen.y(), normal_eigen.z()).normalized();

        // 짐벌락 체크: Y_hint가 Z축과 너무 가까우면 (거의 평행하면)
        if (qAbs(QVector3D::dotProduct(Y_axis_hint, Z_axis)) > 0.95f) {
            qWarning() << "[ICP] Gimbal lock: PCA Normal is parallel to Grasp Axis. Using World Up as fallback 'Hint'.";
            // Y_hint를 월드 Z축(Up)으로 대신 사용
            Y_axis_hint = QVector3D(0.0f, 0.0f, 1.0f);
        }

        // 3. X축 계산 (Y_hint와 Z축 모두에 수직인 벡터)
        QVector3D X_axis = QVector3D::crossProduct(Y_axis_hint, Z_axis).normalized();

        // 4. Y축 계산 (Z축과 X축 모두에 수직인 벡터)
        QVector3D Y_axis = QVector3D::crossProduct(Z_axis, X_axis).normalized();

        // 5. 실제 EF 위치 계산 (파지점에서 Z축 방향으로 뒤로 물러남)
        QVector3D ef_position = graspPoint - Z_axis * GRIPPER_Z_OFFSET;

        // 6. 최종 4x4 행렬 생성 (위치 = ef_position)
        QMatrix4x4 graspPose;
        graspPose.setColumn(0, QVector4D(X_axis, 0.0f));
        graspPose.setColumn(1, QVector4D(Y_axis, 0.0f));
        graspPose.setColumn(2, QVector4D(Z_axis, 0.0f));
        graspPose.setColumn(3, QVector4D(ef_position, 1.0f));

        emit requestRawGraspPoseUpdate(graspPose, true);

        m_icpGraspPose = graspPose; // ✨ [수정] 멤버 변수에 계산된 좌표계 저장
        m_showIcpGraspPose = true; // ✨ [수정] 플래그 설정

        qInfo() << "[ICP] Grasp pose calculated and sent to visualizer.";
        qInfo() << "[ICP] Grasp Point (Magenta): " << graspPoint;
        qInfo() << "[ICP] EF Position (Axis Origin): " << ef_position;

    } else {
        emit requestRawGraspPoseUpdate(QMatrix4x4(), false);

        m_icpGraspPose.setToIdentity(); // ✨ [수정] 실패 시 좌표계 리셋
        m_showIcpGraspPose = false; // ✨ [수정] 플래그 리셋

        if(!pca_ok) qWarning() << "[ICP] PCA calculation failed.";
        if(graspPoint.isNull()) qWarning() << "[ICP] Grasp point calculation failed (no points?).";
    }


    // --- 4. 다이얼로그 표시 ---
    m_icpVizDialog->show();
    m_icpVizDialog->raise();
    m_icpVizDialog->activateWindow();

    qInfo() << "[ICP] Displayed" << focusedHandlePoints.size() << " points in new window.";
}

void RealSenseWidget::onMoveToRandomGraspPoseRequested()
{
    qInfo() << "[GRASP] 'Grasp Handle' move requested (with 5cm approach).";

    if (!m_showRandomGraspPose || m_randomGraspPose.isIdentity()) {
        qWarning() << "[GRASP] Move failed: No random grasp pose calculated. Press 'View handle Plot' first.";
        return;
    }

    // 1. 최종 파지 위치(m) 및 방향(deg) 계산
    QVector3D graspPos_m = m_randomGraspPose.column(3).toVector3D();
    QMatrix3x3 rotMat = m_randomGraspPose.toGenericMatrix<3,3>();

    QVector3D graspOriZYZ = rotationMatrixToEulerAngles(rotMat, "ZYZ");
    float cmdA = graspOriZYZ.x() + 180.0f;
    float cmdB = -graspOriZYZ.y();
    float cmdC = graspOriZYZ.z() + 180.0f;
    while(cmdA > 180.0f) cmdA -= 360.0f; while(cmdA <= -180.0f) cmdA += 360.0f;
    while(cmdB > 180.0f) cmdB -= 360.0f; while(cmdB <= -180.0f) cmdB += 360.0f;
    while(cmdC > 180.0f) cmdC -= 360.0f; while(cmdC <= -180.0f) cmdC += 360.0f;
    QVector3D robotCmdOri_deg(cmdA, cmdB, cmdC);

    // 2. 접근(Approach) 위치 계산 (5cm 뒤)
    QVector3D ef_z_axis = m_randomGraspPose.column(2).toVector3D().normalized();
    float approach_distance_m = 0.05f; // 5cm
    QVector3D approachPos_m = graspPos_m - (ef_z_axis * approach_distance_m);

    // 3. mm 단위로 변환
    QVector3D robotGraspPos_mm = graspPos_m * 1000.0f;
    QVector3D robotApproachPos_mm = approachPos_m * 1000.0f;


    // --- ✨ [추가] 4. 이동 요청 전 IK 체크 수행 ---
    qDebug() << "[GRASP] Checking reachability before emitting move request...";

    // (A) 최종 파지 자세 체크
    if (!checkPoseReachable(robotGraspPos_mm, robotCmdOri_deg)) {
        qWarning() << "[GRASP] ❌ Final grasp pose is UNREACHABLE. Move Canceled.";
        qWarning() << "  - Pos(mm):" << robotGraspPos_mm << "Ori(deg):" << robotCmdOri_deg;
        qWarning() << "[GRASP] Please press 'View handle Plot' again to find a new pose.";
        // 참고: 이 버튼은 '다음' 물체를 자동으로 시도하지 않습니다.
        // 'View handle Plot'을 다시 눌러야 새 자세를 찾습니다.
        return;
    }

    // (B) 접근 자세 체크
    if (!checkPoseReachable(robotApproachPos_mm, robotCmdOri_deg)) {
        qWarning() << "[GRASP] ❌ Approach pose is UNREACHABLE. Move Canceled.";
        qWarning() << "  - Pos(mm):" << robotApproachPos_mm << "Ori(deg):" << robotCmdOri_deg;
        qWarning() << "[GRASP] Please press 'View handle Plot' again to find a new pose.";
        return;
    }

    qInfo() << "[GRASP] ✅ Both poses are reachable. Emitting move request.";
    // --- [추가] 종료 ---


    qInfo() << "[GRASP] Requesting Approach-Then-Grasp Sequence:";
    qInfo() << "  - 1. Approach Pos (mm):" << robotApproachPos_mm;
    qInfo() << "  - 2. Final Pos (mm):"    << robotGraspPos_mm;
    qInfo() << "  - Cmd Rot (A, B, C deg):" << robotCmdOri_deg;

    // 5. 시그널 발생 (이제 IK 체크가 완료된 상태)
    emit requestApproachThenGrasp(robotApproachPos_mm, robotGraspPos_mm, robotCmdOri_deg);
}
void PointCloudWidget::drawRawGraspPoint()
{
    if (!m_isRawVizMode || !m_showRawGraspPoint) return;

    GLUquadric* quadric = gluNewQuadric();
    if(quadric) {
        gluQuadricNormals(quadric, GLU_SMOOTH);

        glColor3f(1.0f, 0.0f, 1.0f); // 자홍색(Magenta/Pink)
        glPushMatrix();
        glTranslatef(m_rawGraspPoint.x(), m_rawGraspPoint.y(), m_rawGraspPoint.z());
        gluSphere(quadric, 0.008, 16, 16); // 8mm radius
        glPopMatrix();

        gluDeleteQuadric(quadric);
    }
}
void PointCloudWidget::drawRawGraspPoseAxis()
{
    if (!m_isRawVizMode || !m_showRawGraspPose) return;

    glPushMatrix();

    // m_rawGraspPose는 이제 (회전 + EF의 실제 위치)를 모두 포함합니다.
    // glTranslate를 따로 호출할 필요 없이, 행렬 전체를 적용합니다.
    glMultMatrixf(m_rawGraspPose.constData());

    // EF 위치에서 5cm 크기로 좌표축을 그립니다.
    drawAxes(0.05f, 3.0f);

    glPopMatrix();
}
void RealSenseWidget::onMoveToIcpGraspPoseRequested()
{
    qInfo() << "[GRASP ICP] 'Grasp Handle' (ICP Pose) move requested.";

    if (!m_showIcpGraspPose || m_icpGraspPose.isIdentity()) {
        qWarning() << "[GRASP ICP] Move failed: No ICP grasp pose calculated. Press 'ICP' button first.";
        return;
    }

    // 1. 최종 파지 위치(m) 및 방향(deg) 계산
    QVector3D graspPos_m = m_icpGraspPose.column(3).toVector3D();
    QMatrix3x3 rotMat = m_icpGraspPose.toGenericMatrix<3,3>();

    QVector3D graspOriZYZ = rotationMatrixToEulerAngles(rotMat, "ZYZ");
    float cmdA = graspOriZYZ.x() + 180.0f;
    float cmdB = -graspOriZYZ.y();
    float cmdC = graspOriZYZ.z() + 180.0f;
    while(cmdA > 180.0f) cmdA -= 360.0f; while(cmdA <= -180.0f) cmdA += 360.0f;
    while(cmdB > 180.0f) cmdB -= 360.0f; while(cmdB <= -180.0f) cmdB += 360.0f;
    while(cmdC > 180.0f) cmdC -= 360.0f; while(cmdC <= -180.0f) cmdC += 360.0f;
    QVector3D robotCmdOri_deg(cmdA, cmdB, cmdC);

    // 2. 접근(Approach) 위치 계산 (5cm "뒤로" - Z축 기준)
    QVector3D ef_z_axis = m_icpGraspPose.column(2).toVector3D().normalized();
    float approach_distance_m = 0.05f; // 5cm

    QVector3D approachPos_m = graspPos_m - (ef_z_axis * approach_distance_m);


    // 3. mm 단위로 변환
    QVector3D robotGraspPos_mm = graspPos_m * 1000.0f;
    QVector3D robotApproachPos_mm = approachPos_m * 1000.0f;


    // --- 4. 이동 요청 전 IK 체크 수행 ---
    qDebug() << "[GRASP ICP] Checking reachability before emitting move request...";

    // (A) 최종 파지 자세 체크
    if (!checkPoseReachable(robotGraspPos_mm, robotCmdOri_deg)) {
        qWarning() << "[GRASP ICP] ❌ Final grasp pose is UNREACHABLE. Move Canceled.";
        qWarning() << "  - Pos(mm):" << robotGraspPos_mm << "Ori(deg):" << robotCmdOri_deg;
        return;
    }

    // (B) 접근 자세 체크
    if (!checkPoseReachable(robotApproachPos_mm, robotCmdOri_deg)) {
        qWarning() << "[GRASP ICP] ❌ Approach pose is UNREACHABLE. Move Canceled.";
        qWarning() << "  - Pos(mm):" << robotApproachPos_mm << "Ori(deg):" << robotCmdOri_deg;
        return;
    }

    qInfo() << "[GRASP ICP] ✅ Both poses are reachable. Emitting move request.";
    // --- [추가] 종료 ---


    qInfo() << "[GRASP ICP] Requesting Approach-Then-Grasp Sequence:";
    qInfo() << "  - 1. Approach Pos (mm):" << robotApproachPos_mm;
    qInfo() << "  - 2. Final Pos (mm):"    << robotGraspPos_mm;
    qInfo() << "  - Cmd Rot (A, B, C deg):" << robotCmdOri_deg;

    // 5. 시그널 발생 (이제 IK 체크가 완료된 상태)
    emit requestApproachThenGrasp(robotApproachPos_mm, robotGraspPos_mm, robotCmdOri_deg);
}
void PointCloudWidget::drawPole()
{
    GLUquadric* quadric = gluNewQuadric();
    if(quadric) {
        gluQuadricNormals(quadric, GLU_SMOOTH);

        // --- 봉 매개변수 (cm -> m 변환) ---
        const float pole_x = 0.48f;
        const float pole_z = 0.34f;
        const float pole_len = 0.15f;
        const float pole_y_center = -0.275f;
        const float pole_y_start = pole_y_center - (pole_len / 2.0f); // Y = -0.35
        // const float pole_y_end = pole_y_center + (pole_len / 2.0f); // (사용 안 됨)
        const float pole_radius = 0.003f; // 1cm (메인 봉)

        // --- 1. 메인 수평 봉 (Y축 평행) ---
        glColor3f(0.3f, 0.3f, 0.3f); // 어두운 회색
        glPushMatrix();
        glTranslatef(pole_x, pole_y_start, pole_z);

        // ✨ [수정] 90.0f -> -90.0f 로 변경하여 봉 방향을 반대로 (-Y 방향)
        glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);

        gluCylinder(quadric, pole_radius, pole_radius, pole_len, 16, 16);
        glPopMatrix();

        // --- 2. 지지대 매개변수 ---
        glColor3f(0.2f, 0.2f, 0.2f); // 더 어두운 회색

        // --- 3. 사각형 판 지지대 (시작 지점) ---
        const float plate_width = 0.30f; // 가로 30cm
        const float plate_height = 0.50f; // 높이 50cm
        const float plate_half_width = plate_width / 2.0f; // 0.15

        // Z=0 (바닥)에서 시작해서 plate_height (50cm) 만큼 위로
        const float new_bottom_z = 0.0f;
        const float new_top_z = new_bottom_z + plate_height; // 0.50

        // 판의 4개 꼭짓점 계산
        // (X-Z 평면에 위치하며, Y좌표는 pole_y_start로 고정)
        QVector3D p1(pole_x - plate_half_width, pole_y_start, new_top_z); // Top-Left (Z=0.50)
        QVector3D p2(pole_x + plate_half_width, pole_y_start, new_top_z); // Top-Right (Z=0.50)
        QVector3D p3(pole_x + plate_half_width, pole_y_start, new_bottom_z); // Bottom-Right (Z=0.0)
        QVector3D p4(pole_x - plate_half_width, pole_y_start, new_bottom_z); // Bottom-Left (Z=0.0)

        // 사각형 판 그리기
        glBegin(GL_QUADS);
        // 앞면
        glVertex3f(p1.x(), p1.y(), p1.z());
        glVertex3f(p2.x(), p2.y(), p2.z());
        glVertex3f(p3.x(), p3.y(), p3.z());
        glVertex3f(p4.x(), p4.y(), p4.z());
        // 뒷면 (양쪽에서 보이도록)
        glVertex3f(p4.x(), p4.y(), p4.z());
        glVertex3f(p3.x(), p3.y(), p3.z());
        glVertex3f(p2.x(), p2.y(), p2.z());
        glVertex3f(p1.x(), p1.y(), p1.z());
        glEnd();

        gluDeleteQuadric(quadric);
    }
}
