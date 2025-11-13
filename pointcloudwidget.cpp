#include "pointcloudwidget.h"
#include "realsensewidget.h" // keyPressEvent에서 부모 위젯 캐스팅에 필요
#include <QDebug>
#include <QMatrix4x4>
#include <QVector3D>
#include <GL/glu.h>
#include <random>
#include <map>
#include <limits>
#include <QMetaObject> // ✨ [추가] invokeMethod 사용을 위해 추가
#include <cmath>
// ===================================================================
// PointCloudWidget 구현
// ===================================================================

PointCloudWidget::PointCloudWidget(QWidget *parent)
    : QOpenGLWidget(parent), m_colorFrame(nullptr), m_showTransformedHandleCloud(false)
{
    setFocusPolicy(Qt::StrongFocus);
    // ✨ [수정] m_panY = -0.3f -> m_panZ = -0.3f로 변경
    //           m_panY는 0.0f로 초기화
    m_yaw = -45.0f; m_pitch = -30.0f; m_panX = 0.0f; m_panY = 0.0f; m_panZ = -0.3f; m_distance = 1.5f;

    m_baseToTcpTransform.setToIdentity();
    const float r[] = { 0.0f, 1.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f };
    QMatrix4x4 rot(r); QMatrix4x4 trans; trans.translate(-0.080, 0.0325, 0.0308);
    m_tcpToCameraTransform = trans * rot;
    m_showPCAAxes = false;        // (이미 있음)
    m_showDebugLookAtPoint = false; // (이미 있음)
    m_showDebugLine = false;      // (이미 있음)
    m_showDebugNormal = false;
    m_showVerticalLine = false; // ✨ [추가]
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

// (realsensewidget.cpp 파일의 약 180행 근처)
void PointCloudWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION); glLoadIdentity();
    float aspect = (float)width() / (height() > 0 ? height() : 1);
    float view_size = m_distance * 0.5f;
    glOrtho(-view_size * aspect, view_size * aspect, -view_size, view_size, -100.0, 100.0);

    glMatrixMode(GL_MODELVIEW); glLoadIdentity();

    // 1. 궤도(Orbit) 회전을 먼저 적용합니다.
    glRotatef(m_pitch, 1.0f, 0.0f, 0.0f);
    glRotatef(m_yaw, 0.0f, 1.0f, 0.0f);

    // ✨ [수정] 2. 3D 패닝(이동) 타겟 지점으로 뷰를 이동합니다.
    glTranslatef(m_panX, m_panY, m_panZ);

    // --- 1. 월드 및 3D 데이터 (깊이 테스트 ON) ---
    drawGrid(2.0f, 20);
    drawAxes(0.2f); // 월드 좌표계

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

        drawRawCentroid();
        drawRawGraspPoint();
        drawRawGraspPoseAxis();

        drawPCAAxes();
        drawVerticalLine(); // ✨ [추가]

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

    // ✨ [추가] 변환된 핸들 클라우드 그리기 (깊이 테스트 ON, 자홍색)
    drawTransformedHandleCloud();


    // --- 2. 3D 오버레이 (깊이 테스트 OFF) ---
    glDisable(GL_DEPTH_TEST); // 깊이 테스트 끄기

    if (!m_isRawVizMode) { // Raw Viz 모드에서는 오버레이를 그리지 않음
        drawGraspingSpheres();
        drawTargetPose(); // 보라색 (컵 파지)
        drawTargetPose_Y_Aligned();
        drawViewPose(); // 노란색 (뷰포인트)
        drawHandleCenterline(); // 핸들 중심선 그리기
        drawRandomGraspPose(); // 주황색 (손잡이 파지)
        drawDebugLookAtPoint();
        drawDebugLine();
        drawDebugNormal();
    } else {
        drawPCAAxes();
        drawDebugNormal();
        drawVerticalLine(); // ✨ [추가]
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
{
    m_lastPos = event->pos();
}

// ✨ [수정] 뷰 중심 회전을 위한 패닝 로직 수정
void PointCloudWidget::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->position().x() - m_lastPos.x();
    int dy = event->position().y() - m_lastPos.y();

    if (event->buttons() & Qt::LeftButton) {
        // 1. 회전 (좌클릭)

        // ✨ [수정] 카메라가 180도 뒤집혔을 때 (Yaw 기준)
        // 마우스 좌우(dx) 입력을 반전시키기 위해 cos(yaw) 값을 이용합니다.

        // 1a. 현재 Yaw 각도를 라디안으로 변환
        float yaw_rad = m_yaw * (M_PI / 180.0f);

        // 1b. Yaw 방향 보정 계수 계산
        // (Yaw 0도 근처 = 1, Yaw 180도 근처 = -1)
        // 90도/270도에서 0이 되어 멈추는 것을 방지하기 위해, 부호(sign)만 사용
        float yaw_direction = (std::cos(yaw_rad) >= 0.0f) ? 1.0f : -1.0f;

        // 1c. 보정된 방향으로 Yaw 값 업데이트
        m_yaw   += dx * 0.5f * yaw_direction;

        // 1d. Pitch(dy)는 상하 반전 문제가 없으므로 그대로 둡니다.
        m_pitch += dy * 0.5f;

    }
    else if (event->buttons() & Qt::RightButton) {
        // 2. 3D 패닝 (우클릭) - (이전 수정안, 올바르게 동작함)

        QMatrix4x4 rotation;
        rotation.rotate(m_pitch, 1.0f, 0.0f, 0.0f);
        rotation.rotate(m_yaw, 0.0f, 1.0f, 0.0f);
        QMatrix4x4 invRotation = rotation.inverted();
        float pan_scale = m_distance * 0.001f;
        QVector3D screenPan(dx * pan_scale, -dy * pan_scale, 0.0f);
        QVector3D worldPan = invRotation * screenPan;

        m_panX += worldPan.x();
        m_panY += worldPan.y();
        m_panZ += worldPan.z();
    }
    m_lastPos = event->pos();
    update();
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
    const float z_off=0.140f, hw=0.040f, jaw_l=0.05f; glLineWidth(3.0f); glColor3f(0.0f,0.0f,0.0f);
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
    if (!m_showViewPose) return;

    glPushMatrix();
    // 1. 뷰 좌표계(노란색 EF)로 이동
    glMultMatrixf(m_viewPoseTransform.constData());

    // 2. 뷰 좌표계(노란색 EF) 그리기
    glPushAttrib(GL_CURRENT_BIT);
    glColor3f(1.0f, 1.0f, 0.0f); // Yellow
    drawAxes(0.1f, 4.0f); // 10cm, 4.0f width
    glPopAttrib();

    // 3. ✨ [사용자 요청] 이 EF 좌표계에 부착된 "카메라" 좌표계 그리기
    glPushMatrix(); // 현재 (노란색 EF) 상태 저장

    // 4. 로컬 TCP -> 카메라 변환 적용
    // (m_tcpToCameraTransform은 생성자에서 초기화됨)
    glMultMatrixf(m_tcpToCameraTransform.constData());

    // 5. 카메라 좌표계(작게) 그리기
    // (5cm, 2.0f width, 기본 R,G,B 색상)
    drawAxes(0.05f, 2.0f);

    glPopMatrix(); // 다시 노란색 EF 좌표계로 복귀

    glPopMatrix(); // 맨 처음 월드 좌표계로 복귀
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

void PointCloudWidget::drawPole()
{
    GLUquadric* quadric = gluNewQuadric();
    if(quadric) {
        gluQuadricNormals(quadric, GLU_SMOOTH);

        // --- 봉 매개변수 (cm -> m 변환) ---
        const float pole_x = 0.68f;
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

// ✨ [새로운 슬롯 구현 추가]
void PointCloudWidget::setPCAAxes(const QVector3D& mean, const QVector3D& pc1, const QVector3D& pc2, const QVector3D& normal, bool show)
{
    m_pcaMeanViz = mean;
    m_pcaPC1Viz = pc1;
    m_pcaPC2Viz = pc2;
    m_pcaNormalViz = normal;
    m_showPCAAxes = show;
    update(); // 화면 갱신 요청
}
// ✨ [새로운 함수 구현 추가]
void PointCloudWidget::drawPCAAxes()
{
    if (!m_showPCAAxes) return;

    float axis_length = 0.05f; // 5cm
    float line_width = 3.0f;

    glPushMatrix();
    glTranslatef(m_pcaMeanViz.x(), m_pcaMeanViz.y(), m_pcaMeanViz.z()); // PCA 중심점으로 이동

    glLineWidth(line_width);
    glBegin(GL_LINES);

    // PC1 (가장 긴 축) - 빨간색
    glColor3f(1.0f, 0.0f, 0.0f); // Red
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(m_pcaPC1Viz.x() * axis_length, m_pcaPC1Viz.y() * axis_length, m_pcaPC1Viz.z() * axis_length);

    // PC2 (두 번째로 긴 축) - 녹색
    glColor3f(0.0f, 1.0f, 0.0f); // Green
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(m_pcaPC2Viz.x() * axis_length, m_pcaPC2Viz.y() * axis_length, m_pcaPC2Viz.z() * axis_length);

    // Normal (표면 법선) - 파란색
    glColor3f(0.0f, 0.0f, 1.0f); // Blue
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(m_pcaNormalViz.x() * axis_length, m_pcaNormalViz.y() * axis_length, m_pcaNormalViz.z() * axis_length);

    glEnd();
    glPopMatrix();
    glLineWidth(1.0f); // 기본값으로 복원
}
void PointCloudWidget::updateDebugLookAtPoint(const QVector3D& point, bool show)
{
    m_debugLookAtPoint = point;
    m_showDebugLookAtPoint = show;
    update();
}
void PointCloudWidget::drawDebugLookAtPoint()
{
    if (!m_showDebugLookAtPoint) return;

    GLUquadric* quadric = gluNewQuadric();
    if(quadric) {
        gluQuadricNormals(quadric, GLU_SMOOTH);

        // Cyan (청록색) 구체
        glColor3f(0.0f, 1.0f, 1.0f);
        glPushMatrix();
        glTranslatef(m_debugLookAtPoint.x(), m_debugLookAtPoint.y(), m_debugLookAtPoint.z());
        // 1.5cm 크기로 크게 그림
        gluSphere(quadric, 0.015, 16, 16);
        glPopMatrix();

        gluDeleteQuadric(quadric);
    }
}
void PointCloudWidget::updateDebugLine(const QVector3D& p1, const QVector3D& p2, bool show)
{
    m_debugLineP1 = p1;
    m_debugLineP2 = p2;
    m_showDebugLine = show;
    update();
}
void PointCloudWidget::drawDebugLine()
{
    if (!m_showDebugLine) return;

    // 자홍색(Magenta) 굵은 선
    glLineWidth(3.0f);
    glColor3f(1.0f, 0.0f, 1.0f);

    glBegin(GL_LINES);
    // P1 (카메라 위치)
    glVertex3f(m_debugLineP1.x(), m_debugLineP1.y(), m_debugLineP1.z());
    // P2 (교점 위치)
    glVertex3f(m_debugLineP2.x(), m_debugLineP2.y(), m_debugLineP2.z());
    glEnd();

    glLineWidth(1.0f); // 라인 두께 복원
}
void PointCloudWidget::updateDebugNormal(const QVector3D& p1, const QVector3D& p2, bool show)
{
    m_debugNormalP1 = p1;
    m_debugNormalP2 = p2;
    m_showDebugNormal = show;
    update();
}

void PointCloudWidget::drawDebugNormal()
{
    if (!m_showDebugNormal) return;

    // Cyan (청록색) 굵은 선
    glLineWidth(3.0f);
    glColor3f(0.0f, 1.0f, 1.0f); // Cyan color

    glBegin(GL_LINES);
    // P1 (시작점 = 자홍색 파지점)
    glVertex3f(m_debugNormalP1.x(), m_debugNormalP1.y(), m_debugNormalP1.z());
    // P2 (끝점)
    glVertex3f(m_debugNormalP2.x(), m_debugNormalP2.y(), m_debugNormalP2.z());
    glEnd();

    glLineWidth(1.0f); // 라인 두께 복원
}

// ✨ [추가] 새로운 슬롯 구현
void PointCloudWidget::updateVerticalLine(const QVector3D& p1, const QVector3D& p2, bool show)
{
    m_verticalLineP1 = p1;
    m_verticalLineP2 = p2;
    m_showVerticalLine = show;
    update();
}

// ✨ [추가] 새로운 그리기 함수 구현
void PointCloudWidget::drawVerticalLine()
{
    if (!m_showVerticalLine) return;

    // 노란색 점선으로 그리기
    glLineWidth(2.0f);
    glColor3f(1.0f, 1.0f, 0.0f); // Yellow
    glLineStipple(1, 0xAAAA); // 점선 패턴 (0b1010101010101010)
    glEnable(GL_LINE_STIPPLE);

    glBegin(GL_LINES);
    glVertex3f(m_verticalLineP1.x(), m_verticalLineP1.y(), m_verticalLineP1.z());
    glVertex3f(m_verticalLineP2.x(), m_verticalLineP2.y(), m_verticalLineP2.z());
    glEnd();

    glDisable(GL_LINE_STIPPLE); // 점선 비활성화
    glLineWidth(1.0f); // 라인 두께 복원
}
// ✨ [추가] 새로운 슬롯 구현
void PointCloudWidget::updateTransformedHandleCloud(const QVector<QVector3D>& points, bool show)
{
    m_transformedHandlePoints = points;
    m_showTransformedHandleCloud = show;
    update(); // 화면 갱신 요청
}

// ✨ [추가] 새로운 그리기 함수 구현
void PointCloudWidget::drawTransformedHandleCloud()
{
    if (!m_showTransformedHandleCloud || m_transformedHandlePoints.isEmpty()) return;

    GLUquadric* quadric = gluNewQuadric();
    if(quadric) {
        gluQuadricNormals(quadric, GLU_SMOOTH);

        // 자홍색(Magenta) 포인트로 그립니다.
        glColor3f(1.0f, 0.0f, 1.0f); // Magenta
        for (const auto& point : m_transformedHandlePoints) {
            glPushMatrix();
            glTranslatef(point.x(), point.y(), point.z());
            // 3mm 반지름, 낮은 디테일로 작게 그립니다.
            gluSphere(quadric, 0.003, 8, 8);
            glPopMatrix();
        }
        gluDeleteQuadric(quadric);
    }
}
