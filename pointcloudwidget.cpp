#include "pointcloudwidget.h"
#include "realsensewidget.h" // keyPressEvent에서 부모 위젯 캐스팅에 필요
#include <QDebug>
#include <QMatrix4x4>
#include <QVector3D>
#include <GL/glu.h>
#include <random>
#include <map>
#include <limits>
#include <QMetaObject>
#include <cmath>
#include <algorithm>

// ===================================================================
// PointCloudWidget 구현
// ===================================================================

PointCloudWidget::PointCloudWidget(QWidget *parent)
    : QOpenGLWidget(parent), m_colorFrame(nullptr), m_showTransformedHandleCloud(false)
{
    setFocusPolicy(Qt::StrongFocus);
    m_yaw = -45.0f; m_pitch = -30.0f; m_panX = 0.0f; m_panY = 0.0f; m_panZ = -0.3f; m_distance = 1.5f;

    m_baseToTcpTransform.setToIdentity();
    const float r[] = { 0.0f, 1.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f };
    QMatrix4x4 rot(r); QMatrix4x4 trans; trans.translate(-0.080, 0.0325, 0.0308);
    m_tcpToCameraTransform = trans * rot;

    m_showPCAAxes = false;
    m_showDebugLookAtPoint = false;
    m_showDebugLine = false;
    m_showDebugNormal = false;
    m_showVerticalLine = false;
    m_showGraspToBodyLine = false;
    m_showOriginalPCAAxes = false;

    // 히트맵 모드 기본값 설정 (1번 모드)
    m_heatmapMode = HeatmapMode::CentroidOnly;
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
        glVertex3f(-halfSize, pos, 0.0f); glVertex3f(halfSize, pos, 0.0f);
        glVertex3f(pos, -halfSize, 0.0f); glVertex3f(pos, halfSize, 0.0f);
    }
    glEnd();
}

// ✨ [수정] Raw 포인트 설정 (데이터 저장 후 업데이트 호출)
QVector3D PointCloudWidget::setRawBaseFramePoints(const QVector<QVector3D>& points)
{
    m_storedRawPoints = points; // 데이터 멤버 변수에 저장
    m_isRawVizMode = !points.isEmpty();

    // 초기화
    m_showRawCentroid = false;
    m_showRawGraspPoint = false;
    m_rawGraspPoint = QVector3D();

    if (points.isEmpty()) {
        makeCurrent();
        m_rawBaseFramePoints.clear();
        doneCurrent();
        update();
        return QVector3D();
    }

    // 저장된 데이터로 히트맵 생성 (현재 모드 적용)
    updateHeatmap();

    return m_rawGraspPoint;
}

// pointcloudwidget.cpp

void PointCloudWidget::updateHeatmap()
{
    if (m_storedRawPoints.isEmpty()) return;

    makeCurrent();
    m_rawBaseFramePoints.clear();
    m_rawBaseFramePoints.reserve(m_storedRawPoints.size() * 6);

    // 1. 무게중심(Centroid) 계산
    QVector3D sum(0.0f, 0.0f, 0.0f);
    for (const auto& p : m_storedRawPoints) sum += p;
    m_rawCentroid = sum / m_storedRawPoints.size();
    m_showRawCentroid = true;

    // 2. 거리 및 높이 데이터 처리 & Reference Point(방향 기준점) 찾기
    float minDist = std::numeric_limits<float>::max();
    float maxDist = -std::numeric_limits<float>::max();
    float minZ = std::numeric_limits<float>::max();
    float maxZ = -std::numeric_limits<float>::max();

    int refPointIndex = -1; // 방향 계산을 위한 고정 기준점 (가장 가까운 점)
    float min_ref_dist = std::numeric_limits<float>::max();

    QVector<float> distances(m_storedRawPoints.size());

    for (int i = 0; i < m_storedRawPoints.size(); ++i) {
        float dist = (m_storedRawPoints[i] - m_rawCentroid).length();
        distances[i] = dist;

        if (dist < minDist) minDist = dist;
        if (dist > maxDist) maxDist = dist;
        float z = m_storedRawPoints[i].z();
        if (z < minZ) minZ = z;
        if (z > maxZ) maxZ = z;

        // 무게중심에 가장 가까운 점을 찾음 (핸들의 배꼽 위치, 불변)
        if (dist < min_ref_dist) {
            min_ref_dist = dist;
            refPointIndex = i;
        }
    }

    float distRange = maxDist - minDist; if (distRange < 1e-6f) distRange = 1.0f;
    float zRange = maxZ - minZ; if (zRange < 1e-6f) zRange = 1.0f;

    // 3. 점수(Intensity) 계산 (EF 위치 결정용)
    QVector<float> raw_intensities(m_storedRawPoints.size());
    float min_raw_intensity = std::numeric_limits<float>::max();
    float max_raw_intensity = -std::numeric_limits<float>::max();
    int bestPointIndex = -1;

    const float W_HEIGHT = 0.7f;
    const float W_PROX = 0.3f;

    for (int i = 0; i < m_storedRawPoints.size(); ++i) {
        float norm_dist = (distances[i] - minDist) / distRange;
        float intensity_prox = 1.0f - norm_dist;
        float norm_height = (m_storedRawPoints[i].z() - minZ) / zRange;

        float raw_intensity = 0.0f;
        if (m_heatmapMode == HeatmapMode::CentroidOnly) {
            raw_intensity = intensity_prox; // 모드 1: 거리만
        } else {
            raw_intensity = (W_PROX * intensity_prox) + (W_HEIGHT * norm_height); // 모드 2: 높이 가중치
        }

        raw_intensities[i] = raw_intensity;
        if (raw_intensity < min_raw_intensity) min_raw_intensity = raw_intensity;
        if (raw_intensity > max_raw_intensity) {
            max_raw_intensity = raw_intensity;
            bestPointIndex = i;
        }
    }

    // 4. 좌표계 생성
    if (bestPointIndex != -1) {
        m_rawGraspPoint = m_storedRawPoints[bestPointIndex]; // 파지 위치 (모드에 따라 변함)
        m_showRawGraspPoint = true;

        // (A) 로컬 노멀 벡터 계산 (파지점 주변)
        QVector<QVector3D> neighbors;
        float radius = 0.01f;
        for(const auto& p : m_storedRawPoints) {
            if (p.distanceToPoint(m_rawGraspPoint) <= radius) {
                neighbors.append(p);
            }
        }

        QVector3D localNormal(0, 0, 1);
        if (neighbors.size() >= 3) {
            Eigen::MatrixXf pointsMat(neighbors.size(), 3);
            Eigen::Vector3f localCentroid(0,0,0);
            for(int k=0; k<neighbors.size(); ++k) {
                pointsMat.row(k) << neighbors[k].x(), neighbors[k].y(), neighbors[k].z();
                localCentroid += pointsMat.row(k);
            }
            localCentroid /= neighbors.size();
            pointsMat.rowwise() -= localCentroid.transpose();

            Eigen::JacobiSVD<Eigen::MatrixXf> svd(pointsMat, Eigen::ComputeThinV);
            Eigen::Vector3f normal = svd.matrixV().col(2);
            localNormal = QVector3D(normal.x(), normal.y(), normal.z());

            m_debugNormalP1 = m_rawGraspPoint;
            m_debugNormalP2 = m_rawGraspPoint + localNormal * 0.05f;
            m_showDebugNormal = true;
        } else {
            m_showDebugNormal = false;
        }

        // -----------------------------------------------------------
        // (B) 좌표계 축 생성 (Y축 고정 로직)
        // -----------------------------------------------------------

        // 1. Y축: 무조건 Handle-Body 직선과 수직 & 수평 유지
        QVector3D Y_axis(0, 1, 0);

        if (m_showGraspToBodyLine) {
            // (1순위) 명시적인 Handle-Body Line 사용 (가장 정확함)
            QVector3D dirBodyToHandle = m_graspToBodyP2 - m_graspToBodyP1;
            dirBodyToHandle.setZ(0.0f);
            if (dirBodyToHandle.lengthSquared() > 1e-6f) {
                dirBodyToHandle.normalize();
                Y_axis = QVector3D::crossProduct(dirBodyToHandle, QVector3D(0, 0, 1)).normalized();
            }
        } else if (refPointIndex != -1) {
            // (2순위) 고정 기준점(RefPoint) 사용 -> 파지점이 바뀌어도 Y축은 안 변함!
            QVector3D dirRadialFixed = (m_storedRawPoints[refPointIndex] - m_rawCentroid);
            dirRadialFixed.setZ(0.0f);
            if (dirRadialFixed.lengthSquared() > 1e-6f) {
                dirRadialFixed.normalize();
                Y_axis = QVector3D::crossProduct(dirRadialFixed, QVector3D(0, 0, 1)).normalized();
            }
        }

        // 2. Z축: 노멀 벡터 반영 & Y축과 직교 & 안쪽 방향
        QVector3D Z_axis = localNormal.normalized();
        Z_axis = (Z_axis - (QVector3D::dotProduct(Z_axis, Y_axis) * Y_axis)).normalized(); // 직교화

        QVector3D dirInward = (m_rawCentroid - m_rawGraspPoint).normalized();
        if (QVector3D::dotProduct(Z_axis, dirInward) < 0) {
            Z_axis = -Z_axis;
        }

        // 3. X축: Y와 Z의 외적
        QVector3D X_axis = QVector3D::crossProduct(Y_axis, Z_axis).normalized();

        // X축 방향 보정 (아래쪽)
        if (X_axis.z() > 0.0f) {
            X_axis = -X_axis;
            Y_axis = -Y_axis;
        }

        // 4. EF 위치 계산
        const float GRIPPER_Z_OFFSET = 0.146f;
        QVector3D efPosition = m_rawGraspPoint - (Z_axis * GRIPPER_Z_OFFSET);

        // 5. 최종 행렬
        QMatrix4x4 pose;
        pose.setToIdentity();
        pose.setColumn(0, QVector4D(X_axis, 0.0f));
        pose.setColumn(1, QVector4D(Y_axis, 0.0f));
        pose.setColumn(2, QVector4D(Z_axis, 0.0f));
        pose.setColumn(3, QVector4D(efPosition, 1.0f));

        m_rawGraspPose = pose;
        m_showRawGraspPose = true;
        emit graspPoseUpdated(m_rawGraspPose);
        qDebug() << "[GraspPose] Frame Stabilized.";
        qDebug() << "  Y(Green):" << Y_axis << " (Should be constant)";

    } else {
        m_showRawGraspPoint = false;
        m_showDebugNormal = false;
        m_showRawGraspPose = false;
    }

    // 5. 색상 매핑
    float raw_intensity_range = max_raw_intensity - min_raw_intensity;
    if (raw_intensity_range < 1e-6f) raw_intensity_range = 1.0f;

    auto getHeatmapColor = [](float norm_val) -> QVector3D {
        float r = 0.0f, g = 0.0f, b = 0.0f;
        if (norm_val < 0.5f) {
            float t = norm_val * 2.0f;
            r = 0.0f; g = t; b = 1.0f - t;
        } else {
            float t = (norm_val - 0.5f) * 2.0f;
            r = t; g = 1.0f - t; b = 0.0f;
        }
        return QVector3D(std::max(0.0f, std::min(1.0f, r)),
                         std::max(0.0f, std::min(1.0f, g)),
                         std::max(0.0f, std::min(1.0f, b)));
    };

    for (int i = 0; i < m_storedRawPoints.size(); ++i) {
        const auto& p = m_storedRawPoints[i];
        m_rawBaseFramePoints.push_back(p.x());
        m_rawBaseFramePoints.push_back(p.y());
        m_rawBaseFramePoints.push_back(p.z());

        float final_intensity = (raw_intensities[i] - min_raw_intensity) / raw_intensity_range;
        QVector3D final_color = getHeatmapColor(final_intensity);

        m_rawBaseFramePoints.push_back(final_color.x());
        m_rawBaseFramePoints.push_back(final_color.y());
        m_rawBaseFramePoints.push_back(final_color.z());
    }

    doneCurrent();
    update();
}

void PointCloudWidget::drawRawCentroid()
{
    if (!m_isRawVizMode || !m_showRawCentroid) return;

    GLUquadric* quadric = gluNewQuadric();
    if(quadric) {
        gluQuadricNormals(quadric, GLU_SMOOTH);
        glColor3f(0.0f, 0.0f, 0.0f); // Black
        glPushMatrix();
        glTranslatef(m_rawCentroid.x(), m_rawCentroid.y(), m_rawCentroid.z());
        gluSphere(quadric, 0.002, 16, 16);
        glPopMatrix();
        gluDeleteQuadric(quadric);
    }
}

// 원본 PCA 축 설정
void PointCloudWidget::setOriginalPCAAxes(const QVector3D& mean, const QVector3D& pc1, const QVector3D& pc2, const QVector3D& normal)
{
    m_pcaMeanOriginal = mean;
    m_pcaPC1Original = pc1;
    m_pcaPC2Original = pc2;
    m_pcaNormalOriginal = normal;
    m_showOriginalPCAAxes = false; // 기본값 숨김
    update();
}

// 원본 PCA 축 그리기 (점선)
void PointCloudWidget::drawOriginalPCAAxes()
{
    if (!m_showOriginalPCAAxes) return;

    float axis_length = 0.08f; // 8cm 고정
    float line_width = 1.5f;

    glPushMatrix();
    glTranslatef(m_pcaMeanOriginal.x(), m_pcaMeanOriginal.y(), m_pcaMeanOriginal.z());

    glLineWidth(line_width);
    glLineStipple(1, 0x00FF);
    glEnable(GL_LINE_STIPPLE);

    glBegin(GL_LINES);
    // PC1 (Red)
    glColor3f(1.0f, 0.5f, 0.5f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(m_pcaPC1Original.x() * axis_length, m_pcaPC1Original.y() * axis_length, m_pcaPC1Original.z() * axis_length);

    // PC2 (Green)
    glColor3f(0.5f, 1.0f, 0.5f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(m_pcaPC2Original.x() * axis_length, m_pcaPC2Original.y() * axis_length, m_pcaPC2Original.z() * axis_length);

    // Normal (Blue)
    glColor3f(0.5f, 0.5f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(m_pcaNormalOriginal.x() * axis_length, m_pcaNormalOriginal.y() * axis_length, m_pcaNormalOriginal.z() * axis_length);
    glEnd();

    glDisable(GL_LINE_STIPPLE);
    glPopMatrix();
    glLineWidth(1.0f);
}

void PointCloudWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION); glLoadIdentity();
    float aspect = (float)width() / (height() > 0 ? height() : 1);
    float view_size = m_distance * 0.5f;
    glOrtho(-view_size * aspect, view_size * aspect, -view_size, view_size, -100.0, 100.0);

    glMatrixMode(GL_MODELVIEW); glLoadIdentity();

    glRotatef(m_pitch, 1.0f, 0.0f, 0.0f);
    glRotatef(m_yaw, 0.0f, 1.0f, 0.0f);
    glTranslatef(m_panX, m_panY, m_panZ);

    drawGrid(2.0f, 20);
    drawAxes(0.2f);

    if (m_isRawVizMode) {
        glPointSize(3.0f);
        if (!m_rawBaseFramePoints.empty()) {
            glEnableClientState(GL_VERTEX_ARRAY); glEnableClientState(GL_COLOR_ARRAY);
            glVertexPointer(3, GL_FLOAT, 6*sizeof(float), m_rawBaseFramePoints.data());
            glColorPointer(3, GL_FLOAT, 6*sizeof(float), (char*)m_rawBaseFramePoints.data() + 3*sizeof(float));
            glDrawArrays(GL_POINTS, 0, m_rawBaseFramePoints.size() / 6);
            glDisableClientState(GL_COLOR_ARRAY); glDisableClientState(GL_VERTEX_ARRAY);
        }
        glPointSize(1.5f);

        drawRawCentroid();
        drawRawGraspPoint();
        drawRawGraspPoseAxis();

        drawPCAAxes();
        drawOriginalPCAAxes();

        drawVerticalLine();
        drawGraspToBodyLine();

    } else {
        glPushMatrix();
        glMultMatrixf(m_baseToTcpTransform.constData());
        drawAxes(0.1f);
        drawGripper();

        glPushMatrix();
        glMultMatrixf(m_tcpToCameraTransform.constData());
        drawAxes(0.05f, 3.0f);
        glPopMatrix();

        glMultMatrixf(m_tcpToCameraTransform.constData());
        if (!m_vertexData.empty()) {
            glEnableClientState(GL_VERTEX_ARRAY); glEnableClientState(GL_COLOR_ARRAY);
            glVertexPointer(3, GL_FLOAT, 6*sizeof(float), m_vertexData.data());
            glColorPointer(3, GL_FLOAT, 6*sizeof(float), (char*)m_vertexData.data() + 3*sizeof(float));
            glDrawArrays(GL_POINTS, 0, m_vertexData.size() / 6);
            glDisableClientState(GL_COLOR_ARRAY); glDisableClientState(GL_VERTEX_ARRAY);
        }
        glPopMatrix();
    }

    drawTransformedHandleCloud();

    glDisable(GL_DEPTH_TEST);

    if (!m_isRawVizMode) {
        drawGraspingSpheres();
        drawTargetPose();
        drawTargetPose_Y_Aligned();
        drawViewPose();
        drawHandleCenterline();
        drawRandomGraspPose();
        drawDebugLookAtPoint();
        drawDebugLine();
        drawDebugNormal();
    } else {
        drawPCAAxes();
        drawDebugNormal();
        drawVerticalLine();
        drawGraspToBodyLine();
    }
    glEnable(GL_DEPTH_TEST);
}

void PointCloudWidget::setRawGraspPose(const QMatrix4x4& pose, bool show)
{
    m_rawGraspPose = pose;
    m_showRawGraspPose = show;
    update();
}

void PointCloudWidget::initializeGL()
{
    initializeOpenGLFunctions();
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glPointSize(1.5f);
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
                    m_vertexData.push_back(color_data[color_idx+2]/255.0f);
                    m_vertexData.push_back(color_data[color_idx+1]/255.0f);
                    m_vertexData.push_back(color_data[color_idx]/255.0f);
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
void PointCloudWidget::wheelEvent(QWheelEvent *event)
{
    if (event->angleDelta().y() > 0) m_distance *= 1.0f / 1.1f; else m_distance *= 1.1f;
    if (m_distance < 0.01f) m_distance = 0.01f; if (m_distance > 50.0f) m_distance = 50.0f;
    update();
}

void PointCloudWidget::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->position().x() - m_lastPos.x();
    int dy = event->position().y() - m_lastPos.y();

    if (event->buttons() & Qt::LeftButton) {
        float yaw_rad = m_yaw * (M_PI / 180.0f);
        float yaw_direction = (std::cos(yaw_rad) >= 0.0f) ? 1.0f : -1.0f;
        m_yaw   += dx * 0.5f * yaw_direction;
        m_pitch += dy * 0.5f;
    }
    else if (event->buttons() & Qt::RightButton) {
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

// ✨ [수정] 키 입력 처리 (모드 전환 및 시각화 토글)
void PointCloudWidget::keyPressEvent(QKeyEvent *event)
{
    if (m_isRawVizMode)
    {
        switch (event->key()) {
        case Qt::Key_1:
            // ✨ 1번 키: 무게 중심 거리만 사용 (Centroid Only)
            m_heatmapMode = HeatmapMode::CentroidOnly;
            updateHeatmap();
            break;
        case Qt::Key_2:
            // ✨ 2번 키: 거리(0.3) + 높이(0.7) 가중치 사용 (Weighted Sum)
            m_heatmapMode = HeatmapMode::Product;
            updateHeatmap();
            break;

        // 기존 1~5번 기능은 3번 이후로 이동하여 충돌 방지
        case Qt::Key_3:
            m_showPCAAxes = !m_showPCAAxes;
            break;
        case Qt::Key_4:
            m_showDebugNormal = !m_showDebugNormal;
            break;
        case Qt::Key_5:
            m_showOriginalPCAAxes = !m_showOriginalPCAAxes;
            break;
        case Qt::Key_6:
            m_showVerticalLine = !m_showVerticalLine;
            break;
        case Qt::Key_7:
            m_showGraspToBodyLine = !m_showGraspToBodyLine;
            break;
        default:
            QOpenGLWidget::keyPressEvent(event);
            return;
        }
        update();
        return;
    }

    // 일반 모드 키 바인딩 (변경 없음)
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
    glColor3f(1.0f, 0.0f, 0.0f);
    for (const auto& point : m_graspingPoints) {
        glPushMatrix(); glTranslatef(point.x(), point.y(), point.z());
        gluSphere(quadric, 0.005, 16, 16); glPopMatrix();
    }
    gluDeleteQuadric(quadric);
}

void PointCloudWidget::drawGraspToBodyLine()
{
    if (!m_showGraspToBodyLine) return;
    glLineWidth(2.0f);
    glColor3f(0.3f, 0.3f, 0.3f);
    glLineStipple(1, 0xCCCC);
    glEnable(GL_LINE_STIPPLE);
    glBegin(GL_LINES);
    glVertex3f(m_graspToBodyP1.x(), m_graspToBodyP1.y(), m_graspToBodyP1.z());
    glVertex3f(m_graspToBodyP2.x(), m_graspToBodyP2.y(), m_graspToBodyP2.z());
    glEnd();
    glDisable(GL_LINE_STIPPLE);
    glLineWidth(1.0f);
}

void PointCloudWidget::updateGraspToBodyLine(const QVector3D& graspPoint, const QVector3D& bodyCenter, bool show)
{
    m_graspToBodyP1 = graspPoint;
    m_graspToBodyP2 = bodyCenter;
    m_showGraspToBodyLine = show;
    update();
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
    glMultMatrixf(m_viewPoseTransform.constData());
    glPushAttrib(GL_CURRENT_BIT);
    glColor3f(1.0f, 1.0f, 0.0f);
    drawAxes(0.1f, 4.0f);
    glPopAttrib();
    glPushMatrix();
    glMultMatrixf(m_tcpToCameraTransform.constData());
    drawAxes(0.05f, 2.0f);
    glPopMatrix();
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
    if (n < 2 || m_handleCenterlineSegmentIds.size() != n) return;
    glLineWidth(8.0f);
    for (int i = 0; i < n - 1; ++i) {
        int segmentId = m_handleCenterlineSegmentIds[i];
        if (segmentId < 0 || segmentId >= 3) segmentId = 0;
        if (segmentId == 0) glColor3f(0.0f, 1.0f, 0.0f);
        else if (segmentId == 1) glColor3f(0.0f, 0.0f, 1.0f);
        else glColor3f(1.0f, 0.0f, 0.0f);
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
    glColor3f(1.0f, 0.5f, 0.0f);
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
        glColor3f(1.0f, 0.0f, 1.0f);
        glPushMatrix();
        glTranslatef(m_rawGraspPoint.x(), m_rawGraspPoint.y(), m_rawGraspPoint.z());
        gluSphere(quadric, 0.002, 16, 16);
        glPopMatrix();
        gluDeleteQuadric(quadric);
    }
}

void PointCloudWidget::drawRawGraspPoseAxis()
{
    if (!m_isRawVizMode || !m_showRawGraspPose) return;
    glPushMatrix();
    glMultMatrixf(m_rawGraspPose.constData());
    drawAxes(0.05f, 3.0f);
    glPopMatrix();
}

void PointCloudWidget::setPCAAxes(const QVector3D& mean, const QVector3D& pc1, const QVector3D& pc2, const QVector3D& normal, bool show)
{
    m_pcaMeanViz = mean;
    m_pcaPC1Viz = pc1;
    m_pcaPC2Viz = pc2;
    m_pcaNormalViz = normal;
    m_showPCAAxes = show;
    update();
}

void PointCloudWidget::drawPCAAxes()
{
    if (!m_showPCAAxes) return;
    float axis_length = 0.05f; float line_width = 3.0f;
    glPushMatrix();
    glTranslatef(m_pcaMeanViz.x(), m_pcaMeanViz.y(), m_pcaMeanViz.z());
    glLineWidth(line_width);
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(m_pcaPC1Viz.x() * axis_length, m_pcaPC1Viz.y() * axis_length, m_pcaPC1Viz.z() * axis_length);
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(m_pcaPC2Viz.x() * axis_length, m_pcaPC2Viz.y() * axis_length, m_pcaPC2Viz.z() * axis_length);
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(m_pcaNormalViz.x() * axis_length, m_pcaNormalViz.y() * axis_length, m_pcaNormalViz.z() * axis_length);
    glEnd();
    glPopMatrix();
    glLineWidth(1.0f);
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
        glColor3f(0.0f, 1.0f, 1.0f);
        glPushMatrix();
        glTranslatef(m_debugLookAtPoint.x(), m_debugLookAtPoint.y(), m_debugLookAtPoint.z());
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
    glLineWidth(3.0f);
    glColor3f(0.0f, 0.0f, 1.0f);
    glBegin(GL_LINES);
    glVertex3f(m_debugLineP1.x(), m_debugLineP1.y(), m_debugLineP1.z());
    glVertex3f(m_debugLineP2.x(), m_debugLineP2.y(), m_debugLineP2.z());
    glEnd();
    glLineWidth(1.0f);
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
    glLineWidth(3.0f);
    glColor3f(0.0f, 1.0f, 1.0f);
    glBegin(GL_LINES);
    glVertex3f(m_debugNormalP1.x(), m_debugNormalP1.y(), m_debugNormalP1.z());
    glVertex3f(m_debugNormalP2.x(), m_debugNormalP2.y(), m_debugNormalP2.z());
    glEnd();
    glLineWidth(1.0f);
}

void PointCloudWidget::updateVerticalLine(const QVector3D& p1, const QVector3D& p2, bool show)
{
    m_verticalLineP1 = p1;
    m_verticalLineP2 = p2;
    m_showVerticalLine = show;
    update();
}

void PointCloudWidget::drawVerticalLine()
{
    if (!m_showVerticalLine) return;
    glLineWidth(2.0f);
    glColor3f(1.0f, 1.0f, 0.0f);
    glLineStipple(1, 0xAAAA);
    glEnable(GL_LINE_STIPPLE);
    glBegin(GL_LINES);
    glVertex3f(m_verticalLineP1.x(), m_verticalLineP1.y(), m_verticalLineP1.z());
    glVertex3f(m_verticalLineP2.x(), m_verticalLineP2.y(), m_verticalLineP2.z());
    glEnd();
    glDisable(GL_LINE_STIPPLE);
    glLineWidth(1.0f);
}

void PointCloudWidget::updateTransformedHandleCloud(const QVector<QVector3D>& points, bool show)
{
    m_transformedHandlePoints = points;
    m_showTransformedHandleCloud = show;
    update();
}

void PointCloudWidget::drawTransformedHandleCloud()
{
    if (!m_showTransformedHandleCloud || m_transformedHandlePoints.isEmpty()) return;
    GLUquadric* quadric = gluNewQuadric();
    if(quadric) {
        gluQuadricNormals(quadric, GLU_SMOOTH);
        glColor3f(1.0f, 0.0f, 1.0f);
        for (const auto& point : m_transformedHandlePoints) {
            glPushMatrix();
            glTranslatef(point.x(), point.y(), point.z());
            gluSphere(quadric, 0.003, 8, 8);
            glPopMatrix();
        }
        gluDeleteQuadric(quadric);
    }
}

void PointCloudWidget::resetVisualizations()
{
    m_showTargetPose = false;
    m_showTargetPose_Y_Aligned = false;
    m_showViewPose = false;
    m_showRandomGraspPose = false;
    m_showRawCentroid = false;
    m_showRawGraspPoint = false;
    m_showRawGraspPose = false;
    m_showPCAAxes = false;
    m_showOriginalPCAAxes = false;
    m_showDebugLookAtPoint = false;
    m_showDebugLine = false;
    m_showDebugNormal = false;
    m_showVerticalLine = false;
    m_showTransformedHandleCloud = false;
    m_showGraspToBodyLine = false;

    update();
}
