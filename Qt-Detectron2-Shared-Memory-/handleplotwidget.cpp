#include "handleplotwidget.h"
#include <QDebug>
#include <QPalette>
#include <QMouseEvent>
#include <cmath>
#include <algorithm> // std::sort, std::max_element
#include <limits> // numeric_limits
#include <numeric> // std::iota

using namespace Eigen;

// ✨ [수정] 세그먼트 색상 정의 (0: Green, 1: Blue, 2: Red)
const QColor HandlePlotWidget::m_segmentColors[3] = {
    Qt::green,  // 첫 번째 구역 (\)
    Qt::blue,   // 중간 구역 (_)
    Qt::red     // 마지막 구역 (/)
};

HandlePlotWidget::HandlePlotWidget(QWidget *parent)
    : QWidget(parent)
    , m_hasCurve(false)
    , m_fitMode(FitMode::Y_on_X)
{
    QPalette pal = palette();
    pal.setColor(QPalette::Window, Qt::white);
    setAutoFillBackground(true);
    setPalette(pal);
    resize(500, 500);
    setWindowTitle("Handle Projection Plot (PCA + Centerline Segmentation)");
}

// ✨ [수정] updateData: 세그먼테이션 함수 호출
void HandlePlotWidget::updateData(const QVector<QPointF>& points)
{
    m_plotData = points;
    m_hasCurve = false;
    m_centerlineData.clear();
    m_smoothedCenterlinePoints.clear();
    m_segmentIds.clear(); // 세그먼트 ID 초기화

    if (m_plotData.size() < 3) {
        qWarning() << "[Fit] Not enough points for 2nd order polynomial fit:" << points.size();
        update();
        return;
    }

    // 데이터 범위 계산 (변경 없음)
    m_minX = m_maxX = m_plotData[0].x(); m_minY = m_maxY = m_plotData[0].y();
    for (const QPointF& p : m_plotData) {
        if (p.x() < m_minX) m_minX = p.x(); if (p.x() > m_maxX) m_maxX = p.x();
        if (p.y() < m_minY) m_minY = p.y(); if (p.y() > m_maxY) m_maxY = p.y();
    }
    m_dataCenter = QPointF((m_minX + m_maxX) / 2.0f, (m_minY + m_maxY) / 2.0f);
    float dataWidth = m_maxX - m_minX; float dataHeight = m_maxY - m_minY;
    dataWidth = qMax(dataWidth, 0.01f) * 1.1f; dataHeight = qMax(dataHeight, 0.01f) * 1.1f;
    m_range = qMax(dataWidth, dataHeight) / 2.0f;

    if (fitBestPolynomial()) {
        calculateCenterlinePoints();
        smoothCenterlinePoints();

        // ✨ [수정] 스무딩된 포인트가 3개 이상이면 기울기 변화 분석 및 세그먼테이션 수행
        if (m_smoothedCenterlinePoints.size() >= 3) {
            analyzeAndSegmentBySlopeChange(); // 함수 이름 변경
        }
    }

    update();
}

// ✨ [수정] paintEvent: drawCenterline 호출 (분기 로직)
void HandlePlotWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    int width = this->width(); int height = this->height();
    m_viewCenter = QPointF(width / 2.0, height / 2.0);
    m_pixelsPerUnit = qMin(width, height) / (2.0f * m_range);

    // 1. 축 그리기 (변경 없음)
    painter.setPen(QPen(Qt::black, 1.5));
    painter.drawLine(0, m_viewCenter.y(), width, m_viewCenter.y());
    painter.drawLine(m_viewCenter.x(), 0, m_viewCenter.x(), height);
    painter.drawText(m_viewCenter.x() + 5, 15, "PC2");
    painter.drawText(width - 40, m_viewCenter.y() - 5, "PC1");

    if (m_plotData.isEmpty()) return;

    // 2. 포인트 그리기 (변경 없음)
    painter.setBrush(Qt::blue); painter.setPen(Qt::NoPen);
    for(const auto& data : m_plotData) {
        painter.drawEllipse(dataToWidget(data), 3, 3);
    }

    // 3. 중심선 그리기 (세그먼테이션 여부 판단)
    drawCenterline(painter);
}

// 데이터 좌표 -> 위젯 좌표 변환 (변경 없음)
QPointF HandlePlotWidget::dataToWidget(const QPointF& dataPoint)
{
    float x_relative = dataPoint.x() - m_dataCenter.x();
    float y_relative = dataPoint.y() - m_dataCenter.y();
    float widget_x = m_viewCenter.x() + x_relative * m_pixelsPerUnit;
    float widget_y = m_viewCenter.y() - y_relative * m_pixelsPerUnit;
    return QPointF(widget_x, widget_y);
}

// 2차 다항식 피팅 (변경 없음)
bool HandlePlotWidget::fitBestPolynomial()
{
    if (m_plotData.size() < 3) { m_hasCurve = false; return false; }
    int n = m_plotData.size();
    Eigen::MatrixXd X_Y(n, 3); Eigen::VectorXd Y_Y(n);
    Eigen::MatrixXd X_X(n, 3); Eigen::VectorXd Y_X(n);
    for (int i = 0; i < n; ++i) {
        double x = m_plotData[i].x(); double y = m_plotData[i].y(); double x2 = x*x; double y2 = y*y;
        X_Y(i, 0) = x2; X_Y(i, 1) = x; X_Y(i, 2) = 1; Y_Y(i) = y;
        X_X(i, 0) = y2; X_X(i, 1) = y; X_X(i, 2) = 1; Y_X(i) = x;
    }
    Eigen::Vector3d coeffs_Y = X_Y.colPivHouseholderQr().solve(Y_Y);
    double error_Y = (Y_Y - X_Y * coeffs_Y).squaredNorm();
    Eigen::Vector3d coeffs_X = X_X.colPivHouseholderQr().solve(Y_X);
    double error_X = (Y_X - X_X * coeffs_X).squaredNorm();
    if (error_Y <= error_X) { m_polyCoeffs = coeffs_Y; m_fitMode = FitMode::Y_on_X; }
    else { m_polyCoeffs = coeffs_X; m_fitMode = FitMode::X_on_Y; }
    m_hasCurve = true; return true;
}

// 곡선 위의 점/벡터 계산 (변경 없음)
std::tuple<QPointF, QVector2D, QVector2D> HandlePlotWidget::getPointVectorsOnCurve(double t)
{
    double a=m_polyCoeffs(0), b=m_polyCoeffs(1), c=m_polyCoeffs(2); QPointF p; QVector2D tan, nml;
    if (m_fitMode == FitMode::Y_on_X) { double x = m_minX + t * (m_maxX - m_minX); double y = a*x*x + b*x + c; p = QPointF(x, y); double dydx = 2*a*x + b; tan = QVector2D(1.0, dydx).normalized(); }
    else { double y = m_minY + t * (m_maxY - m_minY); double x = a*y*y + b*y + c; p = QPointF(x, y); double dxdy = 2*a*y + b; tan = QVector2D(dxdy, 1.0).normalized(); }
    nml = QVector2D(-tan.y(), tan.x()); return {p, nml, tan};
}

// 중심선 계산 (변경 없음)
void HandlePlotWidget::calculateCenterlinePoints()
{
    if (!m_hasCurve) return; m_centerlineData.clear();
    int numSamples = 20; const double rhw = 0.005; const double rhl = (m_range / numSamples) * 0.6;
    for (int i = 0; i <= numSamples; ++i) {
        CenterlineData d; double t = static_cast<double>(i) / numSamples;
        auto [cp, nml, tan] = getPointVectorsOnCurve(t); d.curvePoint=cp; d.normal=nml; d.tangent=tan;
        QPointF tv = tan.toPointF(); QPointF nv = nml.toPointF();
        QPointF p1 = cp + tv*rhl + nv*rhw; QPointF p2 = cp - tv*rhl + nv*rhw;
        QPointF p3 = cp - tv*rhl - nv*rhw; QPointF p4 = cp + tv*rhl - nv*rhw;
        d.rect << p1 << p2 << p3 << p4;
        QVector<double> xV, yV; double minP=std::numeric_limits<double>::max(); double maxP=std::numeric_limits<double>::lowest();
        QPointF minPt=cp; QPointF maxPt=cp;
        for (const QPointF& dp : m_plotData) {
            if (d.rect.containsPoint(dp, Qt::OddEvenFill)) {
                xV.append(dp.x()); yV.append(dp.y());
                double dist = QVector2D::dotProduct(QVector2D(dp - cp), nml);
                if (dist < minP) { minP = dist; minPt = dp; } if (dist > maxP) { maxP = dist; maxPt = dp; }
            }
        }
        if (!xV.isEmpty()) { std::sort(xV.begin(), xV.end()); std::sort(yV.begin(), yV.end()); int mid = xV.size()/2; d.midPoint = QPointF(xV[mid], yV[mid]); d.minPoint=minPt; d.maxPoint=maxPt; d.valid = true; }
        else { d.midPoint=cp; d.minPoint=cp; d.maxPoint=cp; d.valid = true; }
        m_centerlineData.push_back(d);
    }
}

// 중심선 포인트 스무딩 (변경 없음)
void HandlePlotWidget::smoothCenterlinePoints(int windowSize)
{
    m_smoothedCenterlinePoints.clear(); if (m_centerlineData.empty()) return; int n = m_centerlineData.size(); m_smoothedCenterlinePoints.resize(n);
    if (n < windowSize) { for(int i=0; i<n; ++i) m_smoothedCenterlinePoints[i] = m_centerlineData[i].midPoint; return; }
    int hw = windowSize / 2; QVector<QPointF> tmp(n); QVector<bool> valid(n, false); int vc = 0;
    for (int i=0; i<n; ++i) { int s = qMax(0, i-hw); int e = qMin(n-1, i+hw); QPointF sum(0,0); int cnt = 0; for (int k=s; k<=e; ++k) if (m_centerlineData[k].valid) { sum += m_centerlineData[k].midPoint; cnt++; } if (cnt > 0) { tmp[i] = sum / cnt; valid[i] = true; vc++; } }
    if (vc == 0) { qWarning() << "[Smooth] No valid points."; for(int i=0; i<n; ++i) m_smoothedCenterlinePoints[i] = m_centerlineData[i].curvePoint; return; }
    int fv = -1; for(int i=0; i<n; ++i) if(valid[i]) { fv = i; break; } int lv = n-1; for(int i=n-1; i>=0; --i) if(valid[i]) { lv = i; break; }
    for(int i=0; i<fv; ++i) m_smoothedCenterlinePoints[i] = tmp[fv]; for(int i=lv+1; i<n; ++i) m_smoothedCenterlinePoints[i] = tmp[lv];
    for (int i=fv; i<=lv; ++i) { if (valid[i]) { m_smoothedCenterlinePoints[i] = tmp[i]; } else { int gs = i; int pv = i - 1; int nv = gs + 1; while(nv <= lv && !valid[nv]) nv++; QPointF p1 = m_smoothedCenterlinePoints[pv]; QPointF p2 = tmp[nv]; int gl = nv - pv; for(int k = gs; k < nv; ++k) { double t = static_cast<double>(k - pv) / static_cast<double>(gl); m_smoothedCenterlinePoints[k] = p1*(1.0-t) + p2*t; } i = nv - 1; } }
}

// 2차 곡선 그리기 (옵션, 변경 없음)
void HandlePlotWidget::drawPolynomialCurve(QPainter& painter, const QColor& color, double width)
{ if (!m_hasCurve) return; QPainterPath p; int s = 100; for (int i=0; i<=s; ++i) { double t=static_cast<double>(i)/s; QPointF cp=std::get<0>(getPointVectorsOnCurve(t)); QPointF wp=dataToWidget(cp); if(i==0) p.moveTo(wp); else p.lineTo(wp); } painter.setPen(QPen(color, width, Qt::SolidLine)); painter.setBrush(Qt::NoBrush); painter.drawPath(p); }

// 직사각형 영역 그리기 (변경 없음)
void HandlePlotWidget::drawNormalRects(QPainter& painter)
{ if (m_centerlineData.empty()) return; painter.setPen(Qt::NoPen); painter.setBrush(QColor(128,128,128,100)); for(const auto& d : m_centerlineData) if(d.rect.size()==4) { QPolygonF wr; wr << dataToWidget(d.rect[0]) << dataToWidget(d.rect[1]) << dataToWidget(d.rect[2]) << dataToWidget(d.rect[3]); painter.drawPolygon(wr); } }

// ✨ [수정] 기울기 변화 분석 및 세그먼테이션 함수
void HandlePlotWidget::analyzeAndSegmentBySlopeChange()
{
    int n = m_smoothedCenterlinePoints.size();
    if (n < 3) { // 최소 3점 필요
        m_segmentIds.clear(); // 데이터 부족 시 클리어
        return;
    }

    // 1. 각 세그먼트의 기울기(각도) 계산
    QVector<double> angles(n - 1);
    for (int i = 0; i < n - 1; ++i) {
        double dx = m_smoothedCenterlinePoints[i+1].x() - m_smoothedCenterlinePoints[i].x();
        double dy = m_smoothedCenterlinePoints[i+1].y() - m_smoothedCenterlinePoints[i].y();
        angles[i] = atan2(dy, dx); // 라디안 각도
    }

    // 2. 각도 변화량 계산 (변화량은 n-2개 존재)
    QVector<double> angleChanges(n - 2);
    for (int i = 0; i < n - 2; ++i) {
        double diff = angles[i+1] - angles[i];
        // 각도 랩 어라운드 처리 (-pi ~ pi 범위 유지)
        while (diff <= -M_PI) diff += 2 * M_PI;
        while (diff > M_PI) diff -= 2 * M_PI;
        angleChanges[i] = std::abs(diff); // 변화량의 절대값 사용
    }

    // 3. 변화량이 가장 큰 두 지점(인덱스) 찾기
    int break1_idx = -1, break2_idx = -1;
    if (n > 3) { // 변화량이 2개 이상일 때만 두 지점 찾기
        // 첫 번째 최대 변화 지점 찾기
        auto max_iter1 = std::max_element(angleChanges.begin(), angleChanges.end());
        break1_idx = std::distance(angleChanges.begin(), max_iter1);

        // 두 번째 최대 변화 지점 찾기 (첫 번째 최대값 제외)
        double max_val1 = *max_iter1;
        *max_iter1 = -1.0; // 임시로 값을 변경하여 제외
        auto max_iter2 = std::max_element(angleChanges.begin(), angleChanges.end());
        break2_idx = std::distance(angleChanges.begin(), max_iter2);
        *max_iter1 = max_val1; // 값 복원

        // 인덱스 정렬
        if (break1_idx > break2_idx) std::swap(break1_idx, break2_idx);

        qDebug() << "[Segment] Found breaks at indices (angle change):" << break1_idx << break2_idx;

    } else if (n == 3) { // 포인트가 3개면 변화량은 1개, 중간이 구분점
        break1_idx = 0; // 첫 번째 변화
        break2_idx = 0; // (두 번째 구분점 없음)
        qDebug() << "[Segment] Only 3 points, break at index 0";
    }

    // 4. 각 포인트에 세그먼트 ID 할당
    m_segmentIds.clear();
    m_segmentIds.resize(n);

    // break1_idx는 angleChanges 배열에서의 인덱스임.
    // 이는 포인트 i와 i+1 사이의 변화량을 의미함.
    // 따라서 실제 구분점은 포인트 (break1_idx + 1)와 (break2_idx + 1)임.

    int point_break1 = break1_idx + 1;
    int point_break2 = break2_idx + 1;

    if (break1_idx != -1 && break2_idx != -1 && break1_idx != break2_idx) { // 구분점 2개
        for (int i = 0; i < n; ++i) {
            if (i <= point_break1) {
                m_segmentIds[i] = 0; // 첫 번째 구역 (\)
            } else if (i <= point_break2) {
                m_segmentIds[i] = 1; // 중간 구역 (_)
            } else {
                m_segmentIds[i] = 2; // 마지막 구역 (/)
            }
        }
    } else if (break1_idx != -1) { // 구분점 1개 (포인트 3개인 경우)
        for (int i = 0; i < n; ++i) {
            m_segmentIds[i] = (i <= point_break1) ? 0 : 2; // \ 와 / 로만 나눔
        }
        qDebug() << "[Segment] Using only 2 segments (\\ and /)";
    }
    else { // 구분점 없음 (포인트 2개 이하 또는 직선)
        for (int i = 0; i < n; ++i) {
            m_segmentIds[i] = 1; // 모두 중간 구역으로 처리
        }
        qDebug() << "[Segment] Using only 1 segment (_)";
    }

    // 디버깅: 첫 10개 포인트의 세그먼트 ID 확인
    QString first_10_ids = "";
    for(int i=0; i<qMin(n, 10); ++i) first_10_ids += QString::number(m_segmentIds[i]) + " ";
    qDebug() << "[Segment Debug] First 10 point segment IDs:" << first_10_ids;
}


// 단색 중심선 그리기 (기존 로직, 변경 없음)
void HandlePlotWidget::drawSimpleCenterline(QPainter& painter)
{
    if (m_smoothedCenterlinePoints.size() < 2) return;
    QPainterPath path;
    path.moveTo(dataToWidget(m_smoothedCenterlinePoints[0]));
    for(int i = 1; i < m_smoothedCenterlinePoints.size(); ++i) {
        path.lineTo(dataToWidget(m_smoothedCenterlinePoints[i]));
    }
    painter.setPen(QPen(Qt::magenta, 2.5, Qt::SolidLine));
    painter.setBrush(Qt::NoBrush);
    painter.drawPath(path);
}

// ✨ [수정] drawCenterline: 세그먼트 ID 기반으로 그리기
void HandlePlotWidget::drawCenterline(QPainter &painter)
{
    int n = m_smoothedCenterlinePoints.size();
    // 세그먼테이션 데이터가 없거나, 포인트 개수가 안 맞으면 기존 방식(단색)으로 그리기
    if (m_segmentIds.isEmpty() || m_segmentIds.size() != n || n < 2)
    {
        drawSimpleCenterline(painter);
        return;
    }

    // 각 선분(segment)을 해당 시작점의 세그먼트 색상으로 그림
    for (int i = 0; i < n - 1; ++i)
    {
        int segmentId = m_segmentIds[i]; // 현재 선분의 시작점의 세그먼트 ID 사용
        if (segmentId < 0 || segmentId >= 3) {
            qWarning() << "[Draw Warn] Invalid segment ID" << segmentId << "at index" << i;
            segmentId = 0; // 기본값 녹색 사용
        }

        painter.setPen(QPen(m_segmentColors[segmentId], 2.5, Qt::SolidLine));

        QPointF p1_widget = dataToWidget(m_smoothedCenterlinePoints[i]);
        QPointF p2_widget = dataToWidget(m_smoothedCenterlinePoints[i+1]);

        painter.drawLine(p1_widget, p2_widget);
    }

    // 옵션: 각 포인트 자체도 세그먼트 색상으로 그리기 (디버깅에 유용)
    painter.setPen(Qt::NoPen);
    for(int i = 0; i < n; ++i) {
        int segmentId = m_segmentIds[i];
        if (segmentId < 0 || segmentId >= 3) segmentId = 0;
        painter.setBrush(m_segmentColors[segmentId]);
        painter.drawEllipse(dataToWidget(m_smoothedCenterlinePoints[i]), 2, 2); // 작은 점
    }
}
