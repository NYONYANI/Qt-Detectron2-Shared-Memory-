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
#include "projectionplotwidget.h" // ✨ [추가] 2D 프로젝션 플롯 헤더

// ✨ [추가] IK Check를 위해 DRFL 전역 변수 선언
#include "DRFLEx.h"
using namespace DRAFramework;
extern CDRFLEx GlobalDrfl;

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
    m_newResultAwaitingFrameUpdate(false),
    m_icpVizDialog(nullptr),
    m_icpPointCloudWidget(nullptr),
    m_projectionPlotDialog(nullptr),
    m_projectionPlotWidget(nullptr),
    m_showIcpGraspPose(false),
    m_hasGraspPoseCentroidLine(false)
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
    connect(this, &RealSenseWidget::requestDebugLookAtPointUpdate,
            m_pointCloudWidget, &PointCloudWidget::updateDebugLookAtPoint);
    connect(this, &RealSenseWidget::requestDebugLineUpdate,
            m_pointCloudWidget, &PointCloudWidget::updateDebugLine);
    connect(this, &RealSenseWidget::requestDebugNormalUpdate,
            m_pointCloudWidget, &PointCloudWidget::updateDebugNormal);

    connect(this, &RealSenseWidget::requestTransformedHandleCloudUpdate,
            m_pointCloudWidget, &PointCloudWidget::updateTransformedHandleCloud);
    m_pointCloudWidget->setFocus();
}

RealSenseWidget::~RealSenseWidget()
{
    if (m_timer && m_timer->isActive()) m_timer->stop();
    // 공유 메모리 종료 신호 전송
    if (data_control) { static_cast<char*>(data_control)[OFFSET_SHUTDOWN] = 1; }

    // 기존 플롯 위젯 정리
    qDeleteAll(m_plotWidgets);
    m_plotWidgets.clear();
    delete m_handlePlotWidget;

    // 다이얼로그 정리
    if (m_icpVizDialog) {
        m_icpVizDialog->close();
    }
    if (m_projectionPlotDialog) {
        m_projectionPlotDialog->close();
    }

    // ✨ [추가] Top View Dialog 정리
    if (m_topViewDialog) {
        m_topViewDialog->close();
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

bool RealSenseWidget::checkPoseReachable(const QVector3D& pos_mm, const QVector3D& ori_deg)
{
    float target_posx[6];
    target_posx[0] = pos_mm.x();
    target_posx[1] = pos_mm.y();
    target_posx[2] = pos_mm.z();
    target_posx[3] = ori_deg.x();
    target_posx[4] = ori_deg.y();
    target_posx[5] = ori_deg.z();

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
    qDebug() << "[PLOT] Handle Plot requested. (Show Window:" << showWindow << ")";
    // 1. 상태 초기화
    m_handleCenterline3D.clear();
    m_handleSegmentIds.clear();
    m_randomGraspPose.setToIdentity();
    m_showRandomGraspPose = false;
    m_hasPCAData = false;
    m_selectedHandlePoints3D.clear(); // ✨ [수정] m_selectedHandlePoints3D도 초기화

    if (m_detectionResults.isEmpty() || !m_pointCloudWidget->m_points) {
        qDebug() << "[PLOT] No detection results or point cloud available.";
        emit requestHandleCenterlineUpdate(m_handleCenterline3D, m_handleSegmentIds);
        emit requestRandomGraspPoseUpdate(m_randomGraspPose, m_showRandomGraspPose);
        emit visionTaskComplete(); // ✨ [추가] 데이터 없어도 완료 신호 전송
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
        emit visionTaskComplete(); // ✨ [추가] 완료 신호 전송
        return;
    }

    // 3. 손잡이를 로봇 베이스와 가까운 순서로 정렬
    std::sort(allHandleResults.begin(), allHandleResults.end(), [](const HandleAnalysisResult& a, const HandleAnalysisResult& b) {
        return a.distanceToRobot < b.distanceToRobot;
    });

    // 4. 가장 가까운 손잡이부터 순서대로 IK 체크
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

        // 4d. 계산된 자세가 도달 가능한지 IK 체크
        QVector3D calculatedPos_mm = calculatedPos_m * 1000.0f;
        if (checkPoseReachable(calculatedPos_mm, calculatedOri_deg))
        {
            // 4e. 성공: 도달 가능한 자세를 찾음!
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
            m_selectedHandlePoints3D = handle.handlePoints3D; // ✨ [수정] m_selectedHandlePoints3D 저장

            foundReachablePose = true;
            break; // 루프 종료
        }
        else
        {
            // 4f. 실패: 도달 불가능. 다음 손잡이로 계속
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
        m_selectedHandlePoints3D.clear(); // ✨ [수정] m_selectedHandlePoints3D 클리어
    } else {
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

    outPos_m = tcpPosition;

    // --- 6. 로봇 (A,B,C) 방향 계산 ---
    QMatrix3x3 rotMat = outPose.toGenericMatrix<3,3>();
    QVector3D graspOriZYZ = rotationMatrixToEulerAngles(rotMat, "ZYZ");
    float cmdA = graspOriZYZ.x() + 180.0f;
    float cmdB = -graspOriZYZ.y();
    float cmdC = graspOriZYZ.z() + 180.0f;
    while(cmdA > 180.0f) cmdA -= 360.0f; while(cmdA <= -180.0f) cmdA += 360.0f;
    while(cmdB > 180.0f) cmdB -= 360.0f; while(cmdB <= -180.0f) cmdB += 360.0f;
    while(cmdC > 180.0f) cmdC -= 360.0f; while(cmdC <= -180.0f) cmdC += 360.0f;

    outOri_deg = QVector3D(cmdA, cmdB, cmdC);

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
        m_hasGraspPoseCentroidLine = false;
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

        // Top View 필터링 로직
        float min_z = std::numeric_limits<float>::max();
        float max_z = -std::numeric_limits<float>::max();
        for(const auto& p : body3D) {
            if(p.z() < min_z) min_z = p.z();
            if(p.z() > max_z) max_z = p.z();
        }

        float heightRange = max_z - min_z;
        float topThreshold = max_z - (heightRange * 0.1f);

        QVector<QPointF> body2D;
        QVector<PlotData> bodyPlot;

        for(const auto& p3d : body3D) {
            if (p3d.z() > topThreshold) {
                body2D.append(p3d.toPointF());
                if(showPlot) bodyPlot.append({p3d.toPointF(), Qt::green, "Top"});
            }
        }

        if (body2D.size() < 10) {
            body2D.clear();
            if(showPlot) bodyPlot.clear();
            for(const auto& p3d : body3D) {
                body2D.append(p3d.toPointF());
                if(showPlot) bodyPlot.append({p3d.toPointF(), Qt::green, "Body(Fallback)"});
            }
        }

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
        if(showPlot){
            QVector<PlotData> handlePlot;
            for(const auto& p:handle3D) handlePlot.append({p.toPointF(), Qt::blue, "H"});
            detectedBodyPoints.append(bodyPlot);
            detectedHandlePoints.append(handlePlot);
        }
    }

    m_pointCloudWidget->updateGraspingPoints(graspingPointsForViz);
    qDebug() << "[CALC] Grasping point calculation finished in" << timer.elapsed() << "ms.";

    if (showPlot && !detectedBodyPoints.isEmpty()) {
        for (int i = detectedBodyPoints.size(); i < m_plotWidgets.size(); ++i) m_plotWidgets[i]->hide();
        for (int i = 0; i < detectedBodyPoints.size(); ++i) {
            if (i >= m_plotWidgets.size()) m_plotWidgets.append(new XYPlotWidget());
            m_plotWidgets[i]->updateData(detectedBodyPoints[i], detectedHandlePoints[i]);
            m_plotWidgets[i]->setWindowTitle(QString("Cup %1 Fitting Result (Top Only)").arg(i + 1));
            m_plotWidgets[i]->show(); m_plotWidgets[i]->activateWindow();
        }
    }

    if (m_graspingTargets.isEmpty()) {
        qDebug() << "[CALC] No grasping points calculated.";
        m_pointCloudWidget->updateTargetPoses(QMatrix4x4(), false, QMatrix4x4(), false, QMatrix4x4(), false);
        return false;
    }

    // --------------------------------------------------------------------------------
    // [데이터 저장 및 좌표계 표시 설정]
    // --------------------------------------------------------------------------------

    QVector3D currentTcpPos = m_baseToTcpTransform.column(3).toVector3D(); float minDist = std::numeric_limits<float>::max();
    GraspingTarget bestTarget = m_graspingTargets[0];
    for (const auto& target : m_graspingTargets) { float dist = currentTcpPos.distanceToPoint(target.point); if (dist < minDist) { minDist = dist; bestTarget = target; } }

    float best_grasp_z = bestTarget.point.z();

    // 핵심 데이터 업데이트
    m_bodyCenter3D_bestTarget = QVector3D(bestTarget.circleCenter.x(), bestTarget.circleCenter.y(), best_grasp_z);
    m_handleCentroid3D_bestTarget = QVector3D(bestTarget.handleCentroid.x(), bestTarget.handleCentroid.y(), best_grasp_z);
    m_hasGraspPoseCentroidLine = true;

    qDebug() << "[CALC] Saved Body Center:" << m_bodyCenter3D_bestTarget << " Handle Center:" << m_handleCentroid3D_bestTarget;

    m_calculatedTargetPos_m = bestTarget.point + QVector3D(0, 0, GRIPPER_Z_OFFSET);
    const QVector3D& N = bestTarget.direction;
    float target_rz_rad = atan2(N.y(), N.x()) - (M_PI / 2.0f);
    while (target_rz_rad > M_PI) target_rz_rad -= 2*M_PI; while (target_rz_rad <= -M_PI) target_rz_rad += 2*M_PI;

    QVector3D euler_RxRyRz_deg(0.0f, 179.9f, qRadiansToDegrees(target_rz_rad));
    m_calculatedTargetPose.setToIdentity(); m_calculatedTargetPose.translate(m_calculatedTargetPos_m);
    m_calculatedTargetPose.rotate(euler_RxRyRz_deg.z(), 0, 0, 1);
    m_calculatedTargetPose.rotate(euler_RxRyRz_deg.y(), 0, 1, 0);
    m_calculatedTargetPose.rotate(euler_RxRyRz_deg.x(), 1, 0, 0);

    QMatrix3x3 rotMat = m_calculatedTargetPose.toGenericMatrix<3,3>();
    QVector3D graspOri_deg_ZYZ = rotationMatrixToEulerAngles(rotMat, "ZYZ");
    float cmd_A = graspOri_deg_ZYZ.x() + 180.0f; float cmd_B = -graspOri_deg_ZYZ.y(); float cmd_C = graspOri_deg_ZYZ.z() + 180.0f;
    while(cmd_A>180.0f) cmd_A-=360.0f; while(cmd_A<=-180.0f) cmd_A+=360.0f;
    while(cmd_B>180.0f) cmd_B-=360.0f; while(cmd_B<=-180.0f) cmd_B+=360.0f;
    while(cmd_C>180.0f) cmd_C-=360.0f; while(cmd_C<=-180.0f) cmd_C+=360.0f;
    m_calculatedTargetOri_deg = QVector3D(cmd_A, cmd_B, cmd_C);

    qDebug() << "[CALC] Target Grasp Pose | Pos(m):" << m_calculatedTargetPos_m << "| Ori(A,B,C deg):" << m_calculatedTargetOri_deg;

    // ✨ [수정] 항상 좌표계 축을 표시하지 않음 (false)
    bool showAxis = false;

    m_pointCloudWidget->updateTargetPoses(m_calculatedTargetPose, showAxis, QMatrix4x4(), false, QMatrix4x4(), false);

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
void RealSenseWidget::onCalculateHandleViewPose()
{
    qInfo() << "[VIEW] 'Move View' requested. Calculating Look-At Pose (Geometric Method)...";

    // 1. 데이터 유효성 검사 (기존 데이터 활용 우선)
    bool hasValidData = !m_bodyCenter3D_bestTarget.isNull() && !m_handleCentroid3D_bestTarget.isNull();

    if (!hasValidData) {
        qInfo() << "[VIEW] No existing grasp data found. Attempting to calculate...";

        if (!calculateGraspingPoses(false)) {
            qWarning() << "[VIEW] Grasp Pose calc failed.";
            m_hasCalculatedViewPose = false;
            emit visionTaskComplete();
            return;
        }

        if (m_bodyCenter3D_bestTarget.isNull() || m_handleCentroid3D_bestTarget.isNull()) {
            qWarning() << "[VIEW] Body/Handle center data missing even after calculation.";
            m_hasCalculatedViewPose = false;
            emit visionTaskComplete();
            return;
        }
    } else {
        qInfo() << "[VIEW] Using existing valid Body/Handle data from previous step.";
    }

    // ----------------------------------------------------------------------------------
    // [기하학적 이동 로직]

    // 1. 기준점: 컵 바디 중심
    QVector3D bodyCenter = m_bodyCenter3D_bestTarget;

    // 2. 손잡이 방향 벡터 계산
    QVector3D handleCenter = m_handleCentroid3D_bestTarget;
    QVector3D dirBodyToHandle = (handleCenter - bodyCenter);
    dirBodyToHandle.setZ(0.0f); // 수평 방향만 고려

    if (dirBodyToHandle.length() < 0.001f) {
        qWarning() << "[VIEW] Handle/Body centers are too close. Using default X-axis.";
        dirBodyToHandle = QVector3D(1, 0, 0);
    } else {
        dirBodyToHandle.normalize();
    }

    // 3. 카메라 목표 위치 계산 (이동)
    //    - 손잡이 방향으로 25cm, Z축으로 4cm
    float move_handle_dir_m = 0.25f;
    float move_z_dir_m = 0.2f;

    QVector3D cameraTargetPos = bodyCenter + (dirBodyToHandle * move_handle_dir_m) + QVector3D(0.0f, 0.0f, move_z_dir_m);

    // 컵 반지름 동적 계산
    QVector3D surfacePoint = m_calculatedTargetPos_m - QVector3D(0, 0, GRIPPER_Z_OFFSET);
    float cupRadius = QVector2D(bodyCenter.toVector2D() - surfacePoint.toVector2D()).length();

    if (cupRadius < 0.01f || cupRadius > 0.15f) {
        qWarning() << "[VIEW] Calculated radius" << cupRadius << "m seems invalid. Using default 0.04m.";
        cupRadius = 0.04f;
    } else {
        qInfo() << "[VIEW] Calculated Cup Radius:" << cupRadius << "m";
    }

    // 바라보는 지점 (Look At Target) 계산: 교점
    QVector3D intersectionPoint = bodyCenter + (dirBodyToHandle * cupRadius);
    QVector3D lookAtTarget = intersectionPoint;

    qInfo() << "[VIEW] --- Geometric Calculation ---";
    qInfo() << "  - Body Center:" << bodyCenter;
    qInfo() << "  - Handle Dir:" << dirBodyToHandle;
    qInfo() << "  - Camera Pos:" << cameraTargetPos;
    qInfo() << "  - LookAt Target (Intersection):" << lookAtTarget;


    // 4. LookAt 로직
    QVector3D cameraZ = (lookAtTarget - cameraTargetPos).normalized();
    QVector3D worldUp(0.0f, 0.0f, -1.0f);

    QVector3D cameraX = QVector3D::crossProduct(worldUp, cameraZ).normalized();
    if (cameraX.length() < 0.01f) {
        qWarning() << "[VIEW] Gimbal lock detected. Using alternative Up vector.";
        worldUp = QVector3D(0.0f, -1.0f, 0.0f);
        cameraX = QVector3D::crossProduct(worldUp, cameraZ).normalized();
    }
    QVector3D cameraY = QVector3D::crossProduct(cameraZ, cameraX).normalized();

    QMatrix4x4 lookAtCameraPose;
    lookAtCameraPose.setColumn(0, QVector4D(cameraX, 0.0f));
    lookAtCameraPose.setColumn(1, QVector4D(cameraY, 0.0f));
    lookAtCameraPose.setColumn(2, QVector4D(cameraZ, 0.0f));
    lookAtCameraPose.setColumn(3, QVector4D(cameraTargetPos, 1.0f));

    // 5. 로봇 TCP 포즈 역계산
    QMatrix4x4 cameraToTcpTransform = m_tcpToCameraTransform.inverted();
    QMatrix4x4 required_EF_Pose = lookAtCameraPose * cameraToTcpTransform;

    // 6. 시각화 업데이트
    emit requestDebugLookAtPointUpdate(lookAtTarget, true);

    // ✨ [수정] 긴 직선 대신 축 길이(10cm)만큼만 파란색 선(Look At Vector) 표시
    QVector3D dirView = (lookAtTarget - cameraTargetPos).normalized();
    QVector3D shortLineEnd = cameraTargetPos + (dirView * 0.1f); // 10cm 길이
    //emit requestDebugLineUpdate(cameraTargetPos, shortLineEnd, true);

    // 파지 좌표계(Body Grip Pose)는 그리지 않음 (false)
    m_pointCloudWidget->updateTargetPoses(m_calculatedTargetPose, false,
                                          QMatrix4x4(), false,
                                          required_EF_Pose, true);

    QVector3D viewPos_ef = required_EF_Pose.column(3).toVector3D();
    m_calculatedViewPos_mm = viewPos_ef * 1000.0f;
    m_calculatedViewMatrix = required_EF_Pose;
    m_hasCalculatedViewPose = true;

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
    QMatrix3x3 finalRotMat = m_calculatedViewMatrix.toGenericMatrix<3,3>();
    QVector3D finalOriZYZ = rotationMatrixToEulerAngles(finalRotMat, "ZYZ");
    float f_cmdA = finalOriZYZ.x() + 180.0f; float f_cmdB = -finalOriZYZ.y(); float f_cmdC = finalOriZYZ.z() + 180.0f;
    while(f_cmdA>180.0f) f_cmdA-=360.0f; while(f_cmdA<=-180.0f) f_cmdA+=360.0f;
    while(f_cmdB>180.0f) f_cmdB-=360.0f; while(f_cmdB<=-180.0f) f_cmdB+=360.0f;
    while(f_cmdC>180.0f) f_cmdC-=360.0f; while(f_cmdC<=-180.0f) f_cmdC+=360.0f;

    QVector3D finalCmdOri_deg(f_cmdA, f_cmdB, f_cmdC);
    QVector3D finalPos_mm = m_calculatedViewPos_mm;


    // --- 2. Get Current Pose (from last update) ---
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
    QVector3D intermediatePos_mm = currentPos_mm + QVector3D(200.0f, 0.0f, 0.0f);
    QVector3D intermediateCmdOri_deg = currentCmdOri_deg;


    // --- 4. Emit Move Commands (Queued) ---
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
        // 1. 프레임 대기 및 가져오기
        rs2::frameset frames = m_pipeline.wait_for_frames(1000);
        if (!frames) return;

        frames = m_align.process(frames);
        rs2::video_frame color = frames.get_color_frame();
        rs2::depth_frame depth = frames.get_depth_frame();
        if (!color || !depth) return;

        // 2. 필터 적용 (노이즈 제거 등)
        if (m_isDenoisingOn) {
            depth = m_dec_filter.process(depth);
            depth = m_depth_to_disparity.process(depth);
            depth = m_spat_filter.process(depth);
            depth = m_disparity_to_depth.process(depth);
        }

        // 3. 포인트 클라우드 계산
        m_pointcloud.map_to(color);
        rs2::points points = m_pointcloud.calculate(depth);

        // UV 매핑 테이블 업데이트 (2D 좌표 <-> 3D 포인트 인덱스 변환용)
        const rs2::texture_coordinate* tex = points.get_texture_coordinates();
        m_uv_to_point_idx.assign(IMAGE_WIDTH * IMAGE_HEIGHT, -1);
        for (size_t i = 0; i < points.size(); ++i) {
            int u = static_cast<int>(tex[i].u * IMAGE_WIDTH + 0.5f);
            int v = static_cast<int>(tex[i].v * IMAGE_HEIGHT + 0.5f);
            if (u >= 0 && u < IMAGE_WIDTH && v >= 0 && v < IMAGE_HEIGHT)
                m_uv_to_point_idx[v * IMAGE_WIDTH + u] = i;
        }

        // 4. 변환 행렬 업데이트
        m_pointCloudWidget->setTransforms(m_baseToTcpTransform, m_tcpToCameraTransform);

        // 5. 포인트 클라우드 필터링용 마스크 생성 (글자 제외)
        QImage maskOverlay(color.get_width(), color.get_height(), QImage::Format_ARGB32_Premultiplied);
        maskOverlay.fill(Qt::transparent);

        // ✨ [수정] false 전달: 포인트 클라우드 마스킹 시 텍스트(글자)는 그리지 않음
        if (!m_detectionResults.isEmpty())
            drawMaskOverlay(maskOverlay, m_detectionResults, false);

        m_pointCloudWidget->updatePointCloud(points, color, maskOverlay);

        // 6. 화면 표시용 이미지 생성 (글자 포함)
        m_latestFrame = cv::Mat(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP).clone();
        m_currentImage = cvMatToQImage(m_latestFrame);

        // ✨ [수정] true 전달 (또는 생략): 화면에는 텍스트(글자)를 그림
        if (!m_detectionResults.isEmpty())
            drawMaskOverlay(m_currentImage, m_detectionResults, true);

        if(m_isProcessing) {
            QPainter p(&m_currentImage);
            p.setPen(Qt::yellow);
            p.drawText(20, 30, "Processing...");
        }

        m_colorLabel->setPixmap(QPixmap::fromImage(m_currentImage).scaled(m_colorLabel->size(), Qt::KeepAspectRatio));
        m_pointCloudWidget->update();

        // 7. 자동 시퀀스 결과 대기 처리
        if (m_newResultAwaitingFrameUpdate) {
            m_newResultAwaitingFrameUpdate = false;
            qDebug() << "[INFO] Emitting visionTaskComplete() for auto-sequence (AFTER frame update).";
            emit visionTaskComplete();
        }

    } catch (const rs2::error& e) {
        qWarning() << "RS error:" << e.what();
    } catch (...) {
        qWarning() << "Unknown error in updateFrame";
    }
}

void RealSenseWidget::startCameraStream()
{ try { m_pipeline.start(m_config); m_timer->start(33); } catch (const rs2::error &e) { qCritical() << "RS Error:" << e.what(); } }
QImage RealSenseWidget::cvMatToQImage(const cv::Mat &mat)
{ if(mat.empty()) return QImage(); return QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_BGR888).rgbSwapped(); }

void RealSenseWidget::captureAndProcess(bool isAutoSequence)
{
    if (m_isProcessing || m_latestFrame.empty()) {
        qDebug() << "[WARN] Capture failed. (Processing:" << m_isProcessing << "FrameEmpty:" << m_latestFrame.empty() << ")";
        if (isAutoSequence && m_isProcessing) {
            qWarning() << "[SEQ] Auto-sequence capture request IGNORED (already processing). Sequencer will wait.";
        }
        return;
    }

    qDebug() << "[INFO] Sending frame to Python. (AutoSequence:" << isAutoSequence << ")";

    m_isAutoSequenceCapture = isAutoSequence;

    sendImageToPython(m_latestFrame);
    m_isProcessing = true;
    m_processingTimer.start();
    m_resultTimer->start(100);
}

void RealSenseWidget::checkProcessingResult()
{
    if (!m_isProcessing) { m_resultTimer->stop(); return; }
    QJsonArray results = receiveResultsFromPython();

    if (!results.isEmpty()) {
        qDebug() << "[INFO] Received" << results.size() << "results.";
        m_detectionResults = results;
        m_isProcessing = false;
        m_resultTimer->stop();

        if (m_isAutoSequenceCapture) {
            qDebug() << "[INFO] Results received. Flagging for frame update before signaling complete.";
            m_newResultAwaitingFrameUpdate = true;
        } else {
            qDebug() << "[INFO] Manual capture complete. (Not emitting signal)";
        }
        m_isAutoSequenceCapture = false;
    } else {
        if (m_processingTimer.elapsed() > 5000) {
            qWarning() << "[WARN] Python detection timed out after 5 seconds.";
            qWarning() << "[WARN] Forcibly resetting processing flag. Python server may need restart.";

            m_isProcessing = false;
            m_resultTimer->stop();
            m_isAutoSequenceCapture = false;
        }
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

// realsensewidget.cpp

void RealSenseWidget::drawMaskOverlay(QImage &image, const QJsonArray &results, bool drawLabels) {
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

            // ... (RLE 디코딩 및 마스크 이미지 생성 로직은 기존과 동일) ...
            QJsonArray rle=partData["mask_rle"].toArray(); QJsonArray shp=partData["mask_shape"].toArray(); QJsonArray off=partData["offset"].toArray(); QJsonArray center=partData["center"].toArray();
            int H=shp[0].toInt(); int W=shp[1].toInt(); int ox=off[0].toInt(); int oy=off[1].toInt(); int cX=center[0].toInt(); int cY=center[1].toInt(); int cls=partData["cls_id"].toInt();
            QVector<uchar> mask(W*H, 0); int idx=0; uchar val=0;
            for(const QJsonValue& rv : rle) { int len=rv.toInt(); if(idx+len>W*H) len=W*H-idx; if(len>0) memset(mask.data()+idx, val, len); idx+=len; val=(val==0?255:0); if(idx>=W*H) break; }

            QImage mask_img(mask.constData(), W, H, W, QImage::Format_Alpha8);
            QColor maskColor = (cls==1) ? QColor(0,255,0,150) : QColor(0,0,255,150);
            QImage colored(W, H, QImage::Format_ARGB32_Premultiplied); colored.fill(maskColor);

            QPainter p(&colored);
            p.setCompositionMode(QPainter::CompositionMode_DestinationIn);
            p.drawImage(0, 0, mask_img);
            p.end();

            painter.drawImage(ox, oy, colored);

            // ✨ [수정] drawLabels가 true일 때만 텍스트 그리기
            if (drawLabels) {
                painter.setPen(Qt::white);
                painter.setFont(QFont("Arial", 12, QFont::Bold));
                QString label = QString("cup%1: %2").arg(cupIndex).arg(part);
                painter.drawText(cX, cY, label);
            }
        }
        cupIndex++;
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
    qInfo() << "[ICP] 'ICPButton' clicked. Press '1' for Centroid Map, '2' for Product Map.";

    // --------------------------------------------------------------------
    // 1. 포인트 클라우드 추출 및 정렬 (가장 가까운 핸들 찾기)
    // --------------------------------------------------------------------
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


    // --------------------------------------------------------------------
    // 2. 노이즈 필터링 (DBSCAN)
    // --------------------------------------------------------------------
    qInfo() << "[ICP] Applying noise filter to" << focusedHandlePoints.size() << "handle points...";
    std::vector<Point3D> dbscanPoints;
    dbscanPoints.reserve(focusedHandlePoints.size());
    for(int i=0; i<focusedHandlePoints.size(); ++i) {
        dbscanPoints.push_back({
            focusedHandlePoints[i].x(),
            focusedHandlePoints[i].y(),
            focusedHandlePoints[i].z(),
            0,
            i
        });
    }

    const float dbscan_eps = 0.005f;
    const int dbscan_minPts = 10;
    DBSCAN dbscan(dbscan_eps, dbscan_minPts, dbscanPoints);
    dbscan.run();

    std::map<int, int> clusterCounts;
    for(const auto& p : dbscanPoints) {
        if(p.clusterId > 0) {
            clusterCounts[p.clusterId]++;
        }
    }

    int largestClusterId = -1;
    int maxClusterSize = 0;
    for(const auto& pair : clusterCounts) {
        if(pair.second > maxClusterSize) {
            maxClusterSize = pair.second;
            largestClusterId = pair.first;
        }
    }

    QVector<QVector3D> filteredHandlePoints;
    if(largestClusterId != -1) {
        filteredHandlePoints.reserve(maxClusterSize);
        for(const auto& p : dbscanPoints) {
            if(p.clusterId == largestClusterId) {
                filteredHandlePoints.append(focusedHandlePoints[p.originalIndex]);
            }
        }
        qInfo() << "[ICP] Filtered to largest cluster (" << largestClusterId << ") with" << filteredHandlePoints.size() << "points.";
    } else {
        qWarning() << "[ICP] DBSCAN found no clusters. Using original points as fallback.";
        filteredHandlePoints = focusedHandlePoints;
    }

    if (filteredHandlePoints.isEmpty()) {
        qWarning() << "[ICP] Filtering resulted in zero points. Aborting.";
        return;
    }

    // --------------------------------------------------------------------
    // 3. 다이얼로그 생성 및 시그널 연결
    // --------------------------------------------------------------------
    if (m_icpVizDialog == nullptr) {
        m_icpVizDialog = new QDialog(this);
        m_icpVizDialog->setWindowTitle("Focused Handle Heatmap (Press 1 or 2)");
        m_icpVizDialog->setAttribute(Qt::WA_DeleteOnClose);
        m_icpVizDialog->resize(640, 480);

        m_icpPointCloudWidget = new PointCloudWidget(m_icpVizDialog);
        m_icpPointCloudWidget->setTransforms(QMatrix4x4(), QMatrix4x4());

        QVBoxLayout* layout = new QVBoxLayout(m_icpVizDialog);
        layout->addWidget(m_icpPointCloudWidget);
        m_icpVizDialog->setLayout(layout);

        // ✨ [핵심] PointCloudWidget의 좌표계 변경 시그널을 연결
        // 사용자가 키보드 1(무게중심) 또는 2(가중치 곱)를 누르면 이 슬롯이 호출되어
        // 메인 클래스의 파지 목표 지점(m_icpGraspPose)이 업데이트됩니다.
        connect(m_icpPointCloudWidget, &PointCloudWidget::graspPoseUpdated,
                this, [this](const QMatrix4x4& pose){
                    m_icpGraspPose = pose;
                    m_showIcpGraspPose = true;
                    qDebug() << "[ICP] Main Grasp Pose Updated from Visualizer.";
                });

        connect(m_icpVizDialog, &QDialog::finished, [this](){
            m_icpVizDialog = nullptr;
            m_icpPointCloudWidget = nullptr;
            qDebug() << "[ICP] Visualization dialog closed.";
        });

        connect(this, &RealSenseWidget::requestRawGraspPoseUpdate,
                m_icpPointCloudWidget, &PointCloudWidget::setRawGraspPose);
    }

    // --------------------------------------------------------------------
    // 4. 데이터 설정 및 시각화
    // --------------------------------------------------------------------

    // 이 함수 호출 시 내부적으로 updateHeatmap() -> emit graspPoseUpdated()가 실행되어
    // 초기 파지 자세(기본 모드)가 자동으로 m_icpGraspPose에 설정됩니다.
    m_icpPointCloudWidget->setRawBaseFramePoints(filteredHandlePoints);

    // 불필요한 기존 시각화 요소 숨기기
    emit requestRawGraspPoseUpdate(QMatrix4x4(), false);
    emit requestPCAAxesUpdate(QVector3D(), QVector3D(), QVector3D(), QVector3D(), false);
    emit requestOriginalPCAAxesUpdate(QVector3D(), QVector3D(), QVector3D(), QVector3D());
    emit requestDebugNormalUpdate(QVector3D(), QVector3D(), false);
    emit requestVerticalLineUpdate(QVector3D(), QVector3D(), false);
    emit requestGraspToBodyLineUpdate(QVector3D(), QVector3D(), false);

    m_selectedHandlePoints3D = filteredHandlePoints;

    // 2D 플롯 창이 있다면 숨기기
    if (m_projectionPlotDialog) {
        m_projectionPlotDialog->hide();
    }

    // 다이얼로그 표시
    m_icpVizDialog->show();
    m_icpVizDialog->raise();
    m_icpVizDialog->activateWindow();

    qInfo() << "[ICP] Displayed Heatmap and Grasp Pose Visualizer.";
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


    // --- 4. 이동 요청 전 IK 체크 수행 ---
    qDebug() << "[GRASP] Checking reachability before emitting move request...";

    // (A) 최종 파지 자세 체크
    if (!checkPoseReachable(robotGraspPos_mm, robotCmdOri_deg)) {
        qWarning() << "[GRASP] ❌ Final grasp pose is UNREACHABLE. Move Canceled.";
        qWarning() << "  - Pos(mm):" << robotGraspPos_mm << "Ori(deg):" << robotCmdOri_deg;
        qWarning() << "[GRASP] Please press 'View handle Plot' again to find a new pose.";
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

    // 5. 시그널 발생
    // ✨ [오류 수정] 4번째 인자로 빈 QMatrix4x4()를 전달하여 인자 개수 맞춤
    emit requestApproachThenGrasp(robotApproachPos_mm, robotGraspPos_mm, robotCmdOri_deg);
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


    // ✨ [추가] 걸기 자세(Hang Pose)가 계산되었는지 확인
    QMatrix4x4 hang_pose_to_send;
    hang_pose_to_send.setToIdentity(); // 기본값은 Identity

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

    // 5. 시그널 발생 (✨ 수정됨)
    emit requestApproachThenGrasp(robotApproachPos_mm, robotGraspPos_mm, robotCmdOri_deg); // ✨ [추가] hang_pose_to_send 전달
}


void RealSenseWidget::onShowHorizontalGraspVisualization()
{
    qInfo() << "[ICP-H] 'HorizonGripButton' clicked. Finding closest handle (Side Grip, Y=PC2)...";

    if (m_detectionResults.isEmpty() || !m_pointCloudWidget->m_points) {
        qWarning() << "[ICP-H] No detection results or point cloud available. Press 'Capture' first.";
        return;
    }

    const rs2::points& currentPoints = m_pointCloudWidget->m_points;
    const rs2::vertex* vertices = currentPoints.get_vertices();
    const int width = IMAGE_WIDTH; const int height = IMAGE_HEIGHT;
    QMatrix4x4 camToBaseTransform = m_baseToTcpTransform * m_tcpToCameraTransform;

    QList<HandleAnalysisResult> allHandleResults;
    int cupIdxCounter = 0;

    // ... (1. 포인트 클라우드 추출 및 정렬 로직 - 변경 없음) ...
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
        qWarning() << "[ICP-H] No valid 'handle' 3D points found in the current capture.";
        return;
    }

    std::sort(allHandleResults.begin(), allHandleResults.end(), [](const HandleAnalysisResult& a, const HandleAnalysisResult& b) {
        return a.distanceToRobot < b.distanceToRobot;
    });

    QVector<QVector3D> focusedHandlePoints = allHandleResults[0].handlePoints3D;
    qInfo() << "[ICP-H] Found " << allHandleResults.size() << " handles. Focusing on closest one (Cup "
            << allHandleResults[0].cupIndex << ", " << focusedHandlePoints.size() << " points).";


    // --- ✨ 노이즈 필터링 (DBSCAN) 추가 ---
    qInfo() << "[ICP-H] Applying noise filter to" << focusedHandlePoints.size() << "handle points...";
    std::vector<Point3D> dbscanPoints;
    dbscanPoints.reserve(focusedHandlePoints.size());
    for(int i=0; i<focusedHandlePoints.size(); ++i) {
        dbscanPoints.push_back({
            focusedHandlePoints[i].x(),
            focusedHandlePoints[i].y(),
            focusedHandlePoints[i].z(),
            0, i
        });
    }

    const float dbscan_eps = 0.02f; // 2cm
    const int dbscan_minPts = 10;   // 10개
    DBSCAN dbscan_h(dbscan_eps, dbscan_minPts, dbscanPoints);
    dbscan_h.run();

    std::map<int, int> clusterCounts;
    for(const auto& p : dbscanPoints) {
        if(p.clusterId > 0) clusterCounts[p.clusterId]++;
    }

    int largestClusterId = -1;
    int maxClusterSize = 0;
    for(const auto& pair : clusterCounts) {
        if(pair.second > maxClusterSize) {
            maxClusterSize = pair.second;
            largestClusterId = pair.first;
        }
    }

    QVector<QVector3D> filteredHandlePoints;
    if(largestClusterId != -1) {
        filteredHandlePoints.reserve(maxClusterSize);
        for(const auto& p : dbscanPoints) {
            if(p.clusterId == largestClusterId) {
                filteredHandlePoints.append(focusedHandlePoints[p.originalIndex]);
            }
        }
        qInfo() << "[ICP-H] Filtered to largest cluster (" << largestClusterId << ") with" << filteredHandlePoints.size() << "points.";
    } else {
        qWarning() << "[ICP-H] DBSCAN found no clusters. Using original points as fallback.";
        filteredHandlePoints = focusedHandlePoints;
    }

    if (filteredHandlePoints.isEmpty()) {
        qWarning() << "[ICP-H] Filtering resulted in zero points. Aborting.";
        return;
    }
    // --- ✨ 노이즈 필터링 종료 ---


    // ... (2. 다이얼로그 생성 로직 - 변경 없음) ...
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
            qDebug() << "[ICP-H] Visualization dialog closed.";
        });

        connect(this, &RealSenseWidget::requestRawGraspPoseUpdate,
                m_icpPointCloudWidget, &PointCloudWidget::setRawGraspPose);
        connect(this, &RealSenseWidget::requestPCAAxesUpdate,
                m_icpPointCloudWidget, &PointCloudWidget::setPCAAxes);
        connect(this, &RealSenseWidget::requestDebugLineUpdate,
                m_pointCloudWidget, &PointCloudWidget::updateDebugLine);
        connect(this, &RealSenseWidget::requestDebugNormalUpdate,
                m_pointCloudWidget, &PointCloudWidget::updateDebugNormal);
        // ✨ [추가] 수직선 시그널 연결
        connect(this, &RealSenseWidget::requestVerticalLineUpdate,
                m_icpPointCloudWidget, &PointCloudWidget::updateVerticalLine);
    }

    // --- 2B. 2D 프로젝션 다이얼로그 생성 (ICP-H에서는 사용 안 함) ---
    if (m_projectionPlotDialog == nullptr) {
        m_projectionPlotDialog = new QDialog(this);
        m_projectionPlotDialog->setWindowTitle("PCA XY Projection (Outline)");
        m_projectionPlotDialog->setAttribute(Qt::WA_DeleteOnClose);
        m_projectionPlotDialog->resize(500, 500);
        m_projectionPlotWidget = new ProjectionPlotWidget(m_projectionPlotDialog);
        QVBoxLayout* layout = new QVBoxLayout(m_projectionPlotDialog);
        layout->addWidget(m_projectionPlotWidget);
        m_projectionPlotDialog->setLayout(layout);

        connect(m_projectionPlotDialog, &QDialog::finished, [this](){
            m_projectionPlotDialog = nullptr;
            m_projectionPlotWidget = nullptr;
            qDebug() << "[ProjPlot] Projection plot dialog closed.";
        });
    }


    // --- 3. PCA 실행 및 파지 좌표계 계산 ---

    Eigen::Vector3f mean_eigen, pc1_eigen, pc2_eigen, normal_eigen;
    QVector<QPointF> projectedPoints;
    // ✨ [수정] 필터링된 포인트 사용
    bool pca_ok = calculatePCA(filteredHandlePoints, projectedPoints, mean_eigen, pc1_eigen, pc2_eigen, normal_eigen);

    // ✨ [수정] 필터링된 포인트 사용
    QVector3D graspPoint = m_icpPointCloudWidget->setRawBaseFramePoints(filteredHandlePoints);
    QVector3D centroid(mean_eigen.x(), mean_eigen.y(), mean_eigen.z());
    QVector3D global_normal(normal_eigen.x(), normal_eigen.y(), normal_eigen.z());

    if (!graspPoint.isNull() && pca_ok)
    {
        // --- [*** 로직 변경 ***] ---
        // 1. Z축 = PCA Normal (표면 법선, 접근 방향)
        QVector3D Z_axis = QVector3D(normal_eigen.x(), normal_eigen.y(), normal_eigen.z()).normalized();

        // 2. Y축 = PCA PC2 (손잡이의 *짧은* 축, 그리퍼 좌우 방향)
        QVector3D Y_axis = QVector3D(pc2_eigen.x(), pc2_eigen.y(), pc2_eigen.z()).normalized();

        // 3. Z축 방향 보정: Z축이 (파지점->무게중심)을 향하도록 합니다.
        if (QVector3D::dotProduct(Z_axis, centroid - graspPoint) < 0.0f) {
            qInfo() << "[ICP-H] Flipping Z-axis (Normal) to point towards centroid.";
            Z_axis = -Z_axis;
        }

        // 4. X축 계산: (X = Y x Z)
        QVector3D X_axis = QVector3D::crossProduct(Y_axis, Z_axis).normalized();
        // --- [*** 로직 변경 끝 ***] ---

        // 5. 실제 EF 위치 계산 (파지점에서 Z축 방향으로 뒤로 물러남)
        QVector3D ef_position = graspPoint - Z_axis * GRIPPER_Z_OFFSET;

        // 6. 최종 4x4 행렬 생성 (위치 = ef_position)
        QMatrix4x4 graspPose;
        graspPose.setColumn(0, QVector4D(X_axis, 0.0f));
        graspPose.setColumn(1, QVector4D(Y_axis, 0.0f));
        graspPose.setColumn(2, QVector4D(Z_axis, 0.0f));
        graspPose.setColumn(3, QVector4D(ef_position, 1.0f));

        emit requestRawGraspPoseUpdate(graspPose, true);

        emit requestPCAAxesUpdate(centroid,
                                  QVector3D(pc1_eigen.x(), pc1_eigen.y(), pc1_eigen.z()),
                                  QVector3D(pc2_eigen.x(), pc2_eigen.y(), pc2_eigen.z()),
                                  QVector3D(normal_eigen.x(), normal_eigen.y(), normal_eigen.z()),
                                  true);

        m_icpGraspPose = graspPose;
        m_showIcpGraspPose = true;

        qInfo() << "[ICP-H] Side-Grasp (Y=PC2) pose calculated and sent to visualizer.";
        qInfo() << "[ICP-H] Grasp Point (Magenta): " << graspPoint;
        qInfo() << "[ICP-H] EF Position (Axis Origin): " << ef_position;

        // ✨ [추가] 수직선 숨기기 (이 모드에서는 2D 플롯을 사용하지 않음)
        emit requestVerticalLineUpdate(QVector3D(), QVector3D(), false);
        // ✨ [추가] 2D 플롯 클리어
        if (m_projectionPlotWidget) {
            m_projectionPlotWidget->updateData({});
        }

        // ✨ [추가] 변환에 사용할 수 있도록 필터링된 핸들 포인트를 저장
        m_selectedHandlePoints3D = filteredHandlePoints;
        qInfo() << "[ICP-H] Stored" << m_selectedHandlePoints3D.size() << "filtered points for later use.";


    } else {
        emit requestRawGraspPoseUpdate(QMatrix4x4(), false);
        emit requestPCAAxesUpdate(QVector3D(), QVector3D(), QVector3D(), QVector3D(), false);
        emit requestDebugNormalUpdate(QVector3D(), QVector3D(), false);
        m_icpGraspPose.setToIdentity();
        m_showIcpGraspPose = false;
        // ✨ [추가] 수직선 숨기기
        emit requestVerticalLineUpdate(QVector3D(), QVector3D(), false);
        // ✨ [추가] 2D 플롯 클리어
        if (m_projectionPlotWidget) {
            m_projectionPlotWidget->updateData({});
        }
        // ✨ [추가] 실패 시 저장된 포인트도 클리어
        m_selectedHandlePoints3D.clear();


        if(!pca_ok) qWarning() << "[ICP-H] PCA calculation failed.";
        if(graspPoint.isNull()) qWarning() << "[ICP-H] Grasp point calculation failed (no points?).";
    }

    // --- 4. 다이얼로그 표시 ---
    m_icpVizDialog->show();
    m_icpVizDialog->raise();
    m_icpVizDialog->activateWindow();

    // ✨ [수정] 2D 플롯 다이얼로그는 표시하지 않음 (ICP-V만 표시)
    if (m_projectionPlotDialog) {
        m_projectionPlotDialog->hide();
    }

    qInfo() << "[ICP-H] Displayed" << (filteredHandlePoints.isEmpty() ? 0 : filteredHandlePoints.size()) << " points in new window.";
}


void RealSenseWidget::onShowTopViewAnalysis()
{
    qInfo() << "[TopView] Analyzing Cup Body Only (Top Rim Only)...";

    // 1. 데이터 유효성 검사
    if (m_detectionResults.isEmpty() || !m_pointCloudWidget->m_points) {
        qWarning() << "[TopView] No detection results or point cloud available. Press 'Capture' first.";
        return;
    }

    const rs2::points& currentPoints = m_pointCloudWidget->m_points;
    const rs2::vertex* vertices = currentPoints.get_vertices();
    const int width = IMAGE_WIDTH; const int height = IMAGE_HEIGHT;
    QMatrix4x4 camToBaseTransform = m_baseToTcpTransform * m_tcpToCameraTransform;

    // 2. 가장 가까운 컵의 'Body' 포인트만 추출 (핸들 제외)
    const QJsonValue &cupValue = m_detectionResults.first();
    QJsonObject cupResult = cupValue.toObject();

    QVector<QVector3D> cupBodyPoints;

    // 'body' 파트만 타겟으로 설정
    QStringList targetParts = {"body"};

    for (const QString& part : targetParts) {
        if (!cupResult.contains(part) || !cupResult[part].isObject()) {
            qWarning() << "[TopView] No 'body' detected in the first cup.";
            return;
        }

        QJsonObject partData = cupResult[part].toObject();
        QJsonArray rle = partData["mask_rle"].toArray();
        QJsonArray shape = partData["mask_shape"].toArray();
        QJsonArray offset = partData["offset"].toArray();
        int H = shape[0].toInt(); int W = shape[1].toInt();
        int ox = offset[0].toInt(); int oy = offset[1].toInt();

        QVector<uchar> mask_buffer(W * H, 0); int idx = 0; uchar val = 0;
        for(const QJsonValue& run_val : rle) {
            int len = run_val.toInt(); if(idx + len > W * H) len = W * H - idx;
            if(len > 0) memset(mask_buffer.data() + idx, val, len);
            idx += len; val = (val == 0 ? 255 : 0); if(idx >= W * H) break;
        }

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
                            // Z필터가 켜져있다면 바닥 노이즈 제거
                            if (m_pointCloudWidget->m_isZFiltered && p_base.z() <= 0) continue;

                            cupBodyPoints.append(p_base);
                        }
                    }
                }
            }
        }
    }

    if (cupBodyPoints.isEmpty()) {
        qWarning() << "[TopView] Extracted 0 points for cup body.";
        return;
    }

    // 3. 높이(Z) 기준으로 정렬 및 상단부 추출
    std::sort(cupBodyPoints.begin(), cupBodyPoints.end(), [](const QVector3D& a, const QVector3D& b){
        return a.z() < b.z();
    });

    float minZ = cupBodyPoints.first().z();
    float maxZ = cupBodyPoints.last().z();
    float heightRange = maxZ - minZ;

    qInfo() << "[TopView] Body Height Range:" << minZ << "~" << maxZ << "(Height:" << heightRange << "m)";

    // 상단 10% 임계값
    float topThreshold = maxZ - (heightRange * 0.1f);

    QVector<PlotData> topPlotData;

    for (const QVector3D& p : cupBodyPoints) {
        // 상단 10% (녹색)만 수집
        if (p.z() > topThreshold) {
            topPlotData.append({p.toPointF(), Qt::green, "Top 10%"});
        }
    }

    qInfo() << "[TopView] Points - Top(10%):" << topPlotData.size();

    // 4. 결과 시각화 (XYPlotWidget)
    if (m_topViewDialog == nullptr) {
        m_topViewDialog = new QDialog(this);
        m_topViewDialog->setWindowTitle("Cup Top Projection");
        m_topViewDialog->setAttribute(Qt::WA_DeleteOnClose);
        m_topViewDialog->resize(500, 500);

        QVBoxLayout* layout = new QVBoxLayout(m_topViewDialog);
        m_topViewPlotWidget = new XYPlotWidget(m_topViewDialog);
        layout->addWidget(m_topViewPlotWidget);

        connect(m_topViewDialog, &QDialog::finished, [this](){
            m_topViewDialog = nullptr;
            m_topViewPlotWidget = nullptr;
        });
    }

    // ✨ [수정] 두 번째 인자에 빈 벡터를 전달하여 중간 지점 데이터 삭제
    m_topViewPlotWidget->updateData(topPlotData, QVector<PlotData>());
    m_topViewPlotWidget->setWindowTitle(QString("Top Rim Analysis (Top 10%)"));

    m_topViewDialog->show();
    m_topViewDialog->raise();
    m_topViewDialog->activateWindow();
}
void RealSenseWidget::onMoveToTopViewPose()
{
    qInfo() << "[TopView] 'MoveTopButton' pressed. Calculating Camera Pose...";

    // 1. 파지 자세가 계산되어 있는지 확인 (없으면 계산 시도)
    if (m_calculatedTargetPose.isIdentity()) {
        qWarning() << "[TopView] Grasp pose not ready. Calculating now...";
        if (!calculateGraspingPoses(false)) {
            qWarning() << "[TopView] Failed to calculate grasp pose.";
            return;
        }
    }

    // 2. 오프셋 계산
    float circleRadius = 0.04f; // 컵의 반지름 (4cm 가정)

    QVector3D graspPoint = m_calculatedTargetPose.column(3).toVector3D();
    QVector3D bodyCenter = m_bodyCenter3D_bestTarget; // calculateGraspingPoses에서 저장된 컵 중심

    // Body Center 정보가 없을 경우에 대한 안전장치
    if(bodyCenter.isNull()) {
        qWarning() << "[TopView] Body center info missing. Using default +Y offset.";
    }

    // --- [핵심 로직] Y축 동적 오프셋 방향 결정 ---
    // 파지점(Grasp Point)에서 몸통 중심(Body Center)으로 향하는 벡터 계산
    QVector3D vecGraspToCenter = (bodyCenter - graspPoint).normalized();

    // 파지 좌표계(TCP)의 Y축 벡터 (Rotation Matrix의 2번째 컬럼)
    QVector3D graspY_axis_ef = m_calculatedTargetPose.column(1).toVector3D().normalized();

    // 두 벡터의 내적(Dot Product)을 통해 방향 일치 여부 확인
    float dot_Y = QVector3D::dotProduct(graspY_axis_ef, vecGraspToCenter);

    float offset_y = 0.0f;
    if (dot_Y > 0) {
        // 내적이 양수: Y축이 몸통 중심을 향하고 있음 -> +Radius 방향으로 이동
        offset_y = circleRadius;
        qInfo() << "[TopView] Dynamic Y Offset: Inward (+Y) (dot=" << dot_Y << ")";
    } else {
        // 내적이 음수: Y축이 몸통 바깥을 향하고 있음 -> -Radius 방향으로 이동
        offset_y = -circleRadius;
        qInfo() << "[TopView] Dynamic Y Offset: Outward (-Y) (dot=" << dot_Y << ")";
    }

    // 3. 카메라 목표 좌표계 계산 (파지 좌표계 복사 후 이동)
    QMatrix4x4 cameraTargetPose = m_calculatedTargetPose;

    // 3-1) 파지 좌표계 기준 Y축으로 반지름만큼 이동 (컵의 중심 위로 이동)
    cameraTargetPose.translate(0.0f, offset_y, 0.0f);

    // 3-2) 월드 좌표계 기준 Z축(위쪽)으로 10cm 이동 (높이 확보)
    // (LookAt 회전 없이 위치만 이동)
    float current_world_z = cameraTargetPose(2, 3);
    cameraTargetPose(2, 3) = current_world_z + 0.1f; // 0.1m = 10cm

    qInfo() << "[TopView] Applied Z-Axis Offset (+10cm). New Z:" << cameraTargetPose(2, 3);


    // 4. 로봇(TCP) 목표 자세 역계산 (T_base_tcp = T_base_camera_target * T_camera_tcp)
    QMatrix4x4 robotTargetPose = cameraTargetPose * m_tcpToCameraTransform.inverted();

    // 5. 이동 명령 생성 및 전송
    QVector3D targetPos_m = robotTargetPose.column(3).toVector3D();
    QVector3D targetPos_mm = targetPos_m * 1000.0f;

    QMatrix3x3 rotMat = robotTargetPose.toGenericMatrix<3,3>();
    QVector3D targetOri_deg = rotationMatrixToEulerAngles(rotMat, "ZYZ");

    // 로봇 명령어 포맷 (A, B, C) 변환 및 정규화
    float cmdA = targetOri_deg.x() + 180.0f;
    float cmdB = -targetOri_deg.y();
    float cmdC = targetOri_deg.z() + 180.0f;

    while(cmdA > 180.0f) cmdA -= 360.0f; while(cmdA <= -180.0f) cmdA += 360.0f;
    while(cmdB > 180.0f) cmdB -= 360.0f; while(cmdB <= -180.0f) cmdB += 360.0f;
    while(cmdC > 180.0f) cmdC -= 360.0f; while(cmdC <= -180.0f) cmdC += 360.0f;

    QVector3D finalOri_deg(cmdA, cmdB, cmdC);

    qInfo() << "[TopView] Sending Robot Move Command.";
    qInfo() << "  - Target Pos(mm):" << targetPos_mm;
    qInfo() << "  - Target Ori(deg):" << finalOri_deg;

    emit requestRobotMove(targetPos_mm, finalOri_deg);
}
