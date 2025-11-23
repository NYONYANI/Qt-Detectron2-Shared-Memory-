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
    m_showHangCenter = false;
    m_showOriginalPCAAxes = false; // ✨ [추가] 초기화
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

QVector3D PointCloudWidget::setRawBaseFramePoints(const QVector<QVector3D>& points)
{
    makeCurrent();
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

    m_rawBaseFramePoints.reserve(points.size() * 6);

    QVector3D sum(0.0f, 0.0f, 0.0f);
    for (const auto& p : points) sum += p;
    m_rawCentroid = sum / points.size();
    m_showRawCentroid = true;

    float minDist = FLT_MAX; float maxDist = -FLT_MAX;
    float minZ = FLT_MAX; float maxZ = -FLT_MAX;
    QVector<float> distances(points.size());

    for (int i = 0; i < points.size(); ++i) {
        float dist = (points[i] - m_rawCentroid).length();
        distances[i] = dist;
        if (dist < minDist) minDist = dist;
        if (dist > maxDist) maxDist = dist;
        float z = points[i].z();
        if (z < minZ) minZ = z;
        if (z > maxZ) maxZ = z;
    }

    float distRange = maxDist - minDist; if (distRange < 1e-6f) distRange = 1.0f;
    float zRange = maxZ - minZ; if (zRange < 1e-6f) zRange = 1.0f;

    QVector<float> raw_intensities(points.size());
    float min_raw_intensity = FLT_MAX; float max_raw_intensity = -FLT_MAX;
    int bestPointIndex = -1;

    for (int i = 0; i < points.size(); ++i) {
        float norm_dist = (distances[i] - minDist) / distRange;
        float intensity_prox = 1.0f - norm_dist;
        float norm_height = (points[i].z() - minZ) / zRange;
        float raw_intensity = intensity_prox * norm_height;

        raw_intensities[i] = raw_intensity;
        if (raw_intensity < min_raw_intensity) min_raw_intensity = raw_intensity;
        if (raw_intensity > max_raw_intensity) {
            max_raw_intensity = raw_intensity;
            bestPointIndex = i;
        }
    }

    if (bestPointIndex != -1) {
        m_rawGraspPoint = points[bestPointIndex];
        m_showRawGraspPoint = true;
    }

    float raw_intensity_range = max_raw_intensity - min_raw_intensity;
    if (raw_intensity_range < 1e-6f) raw_intensity_range = 1.0f;

    auto getStandardColor = [](float norm_val) -> QVector3D {
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

    for (int i = 0; i < points.size(); ++i) {
        const auto& p = points[i];
        m_rawBaseFramePoints.push_back(p.x());
        m_rawBaseFramePoints.push_back(p.y());
        m_rawBaseFramePoints.push_back(p.z());

        float final_intensity = (raw_intensities[i] - min_raw_intensity) / raw_intensity_range;
        QVector3D final_color = getStandardColor(final_intensity);

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
        glColor3f(0.0f, 0.0f, 0.0f); // Black
        glPushMatrix();
        glTranslatef(m_rawCentroid.x(), m_rawCentroid.y(), m_rawCentroid.z());
        gluSphere(quadric, 0.002, 16, 16);
        glPopMatrix();
        gluDeleteQuadric(quadric);
    }
}

void PointCloudWidget::updateHangCenterPoint(const QVector3D& point, bool show)
{
    m_hangCenterViz = point;
    m_showHangCenter = show;
    update();
}

void PointCloudWidget::drawHangCenterPoint()
{
    if (!m_showHangCenter) return;
    GLUquadric* quadric = gluNewQuadric();
    if(quadric) {
        gluQuadricNormals(quadric, GLU_SMOOTH);
        glColor3f(0.0f, 0.0f, 0.0f);
        glPushMatrix();
        glTranslatef(m_hangCenterViz.x(), m_hangCenterViz.y(), m_hangCenterViz.z());
        gluSphere(quadric, 0.004, 16, 16);
        glPopMatrix();
        gluDeleteQuadric(quadric);
    }
}

// ✨ [수정] 원본 PCA 축 설정 (길이 인자 제거)
void PointCloudWidget::setOriginalPCAAxes(const QVector3D& mean, const QVector3D& pc1, const QVector3D& pc2, const QVector3D& normal)
{
    m_pcaMeanOriginal = mean;
    m_pcaPC1Original = pc1;
    m_pcaPC2Original = pc2;
    m_pcaNormalOriginal = normal;
    m_showOriginalPCAAxes = false; // 기본값 숨김
    update();
}

// ✨ [수정] 원본 PCA 축 그리기 (점선, 고정 길이)
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

    if (!m_isRawVizMode) {
        drawPole();
    }

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
        drawHangCenterPoint();
        drawRawGraspPoint();
        drawRawGraspPoseAxis();

        drawPCAAxes();
        drawOriginalPCAAxes(); // ✨ [추가]

        drawVerticalLine();
        //drawGraspToBodyLine();

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
        drawHangCenterPoint();
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

void PointCloudWidget::keyPressEvent(QKeyEvent *event)
{
    if (m_isRawVizMode)
    {
        switch (event->key()) {
        case Qt::Key_1:
            m_showPCAAxes = !m_showPCAAxes;
            qDebug() << "[ICP Key] Toggle PCA Axes:" << m_showPCAAxes;
            break;
        case Qt::Key_2:
            m_showDebugNormal = !m_showDebugNormal;
            qDebug() << "[ICP Key] Toggle Normal Vector:" << m_showDebugNormal;
            break;
        case Qt::Key_3:
            m_showVerticalLine = !m_showVerticalLine;
            m_showHangCenter = !m_showHangCenter;
            qDebug() << "[ICP Key] Toggle Vertical Line/Center:" << m_showVerticalLine;
            break;
        case Qt::Key_4:
            m_showGraspToBodyLine = !m_showGraspToBodyLine;
            qDebug() << "[ICP Key] Toggle Grasp-to-Body Line:" << m_showGraspToBodyLine;
            break;
        case Qt::Key_5:
            m_showOriginalPCAAxes = !m_showOriginalPCAAxes;
            qDebug() << "[ICP Key] Toggle Original PCA Axes:" << m_showOriginalPCAAxes;
            break;
        default:
            QOpenGLWidget::keyPressEvent(event);
            return;
        }
        update();
        return;
    }

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

void PointCloudWidget::drawPole()
{
    GLUquadric* quadric = gluNewQuadric();
    if(quadric) {
        gluQuadricNormals(quadric, GLU_SMOOTH);
        const float pole_x = 0.68f; const float pole_z = 0.34f;
        const float pole_y_start = -0.27f; const float pole_len = 0.15f; const float pole_radius = 0.003f;

        glColor3f(0.3f, 0.3f, 0.3f);
        glPushMatrix();
        glTranslatef(pole_x, pole_y_start, pole_z);
        glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
        gluCylinder(quadric, pole_radius, pole_radius, pole_len, 16, 16);
        glPopMatrix();

        glColor3f(0.2f, 0.2f, 0.2f);
        const float plate_width = 0.30f; const float plate_height = 0.50f;
        const float plate_half_width = plate_width / 2.0f;
        const float new_bottom_z = 0.0f; const float new_top_z = new_bottom_z + plate_height;

        QVector3D p1(pole_x - plate_half_width, pole_y_start, new_top_z);
        QVector3D p2(pole_x + plate_half_width, pole_y_start, new_top_z);
        QVector3D p3(pole_x + plate_half_width, pole_y_start, new_bottom_z);
        QVector3D p4(pole_x - plate_half_width, pole_y_start, new_bottom_z);

        glBegin(GL_QUADS);
        glVertex3f(p1.x(), p1.y(), p1.z()); glVertex3f(p2.x(), p2.y(), p2.z());
        glVertex3f(p3.x(), p3.y(), p3.z()); glVertex3f(p4.x(), p4.y(), p4.z());
        glVertex3f(p4.x(), p4.y(), p4.z()); glVertex3f(p3.x(), p3.y(), p3.z());
        glVertex3f(p2.x(), p2.y(), p2.z()); glVertex3f(p1.x(), p1.y(), p1.z());
        glEnd();
        gluDeleteQuadric(quadric);
    }
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
    glColor3f(1.0f, 0.0f, 1.0f);
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
    // 모든 시각화 플래그 끄기
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
    m_showHangCenter = false;
    m_showTransformedHandleCloud = false;
    m_showGraspToBodyLine = false;

    // 데이터도 필요하면 초기화 (선택 사항)
    // m_graspingPoints.clear();

    // 화면 갱신
    update();
}
