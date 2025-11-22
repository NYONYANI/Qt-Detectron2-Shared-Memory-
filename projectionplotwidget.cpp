#include "projectionplotwidget.h"
#include <QDebug>
#include <QPalette>
#include <algorithm> // std::sort
#include <QPainterPath>

// BBox 중심 위치를 저장하기 위한 비공개 멤버 변수
namespace {
QPointF m_bboxCenterForRedDot = QPointF(0, 0);
}

ProjectionPlotWidget::ProjectionPlotWidget(QWidget *parent)
    : QWidget(parent)
{
    QPalette pal = palette();
    pal.setColor(QPalette::Window, Qt::white);
    setAutoFillBackground(true);
    setPalette(pal);
    resize(500, 500);
    setWindowTitle("Aligned Axes Projection Plot (Outline)"); // 창 제목도 변경
}

void ProjectionPlotWidget::updateData(const QVector<QPointF>& projectedPoints)
{
    m_projectedPoints = projectedPoints;
    m_outlinePolygon.clear();

    if (m_projectedPoints.size() < 3) {
        qWarning() << "[ProjPlot] Not enough points for outline:" << m_projectedPoints.size();
        m_dataCenter = QPointF(0,0);
    } else {
        // 데이터 범위 계산
        float minX = m_projectedPoints[0].x(), maxX = m_projectedPoints[0].x();
        float minY = m_projectedPoints[0].y(), maxY = m_projectedPoints[0].y();
        for (const QPointF& p : m_projectedPoints) {
            if (p.x() < minX) minX = p.x(); if (p.x() > maxX) maxX = p.x();
            if (p.y() < minY) minY = p.y(); if (p.y() > maxY) maxY = p.y();
        }

        // 외곽선 중심(빨간 원 위치)을 저장
        QPointF local_bboxCenter = QPointF((minX + maxX) / 2.0f, (minY + maxY) / 2.0f);
        m_bboxCenterForRedDot = local_bboxCenter;

        // m_dataCenter를 중심 (0,0)으로 설정 (플롯의 원점을 Centroid로 고정)
        m_dataCenter = QPointF(0, 0);

        // Range 계산: 0,0을 중심으로 하는 최대 편차 기준
        float maxAbsX = qMax(qAbs(minX), qAbs(maxX));
        float maxAbsY = qMax(qAbs(minY), qAbs(maxY));

        m_range = qMax(maxAbsX, maxAbsY) * 1.1f;
        m_range = qMax(m_range, 0.01f);

        // 외곽선 계산
        calculateConvexHull();
    }
    update();
}

void ProjectionPlotWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    int width = this->width(); int height = this->height();
    m_viewCenter = QPointF(width / 2.0, height / 2.0);
    m_pixelsPerUnit = qMin(width, height) / (2.0f * m_range);

    drawAxes(painter);
    if (m_projectedPoints.isEmpty()) return;
    drawPoints(painter);
    drawOutline(painter);
    drawCenterPoint(painter);
}

QPointF ProjectionPlotWidget::dataToWidget(const QPointF& dataPoint)
{
    float x_relative = dataPoint.x() - m_dataCenter.x();
    float y_relative = dataPoint.y() - m_dataCenter.y();
    // m_dataCenter는 (0,0)이므로, 데이터는 Centroid를 기준으로 그려짐
    float widget_x = m_viewCenter.x() + x_relative * m_pixelsPerUnit;
    float widget_y = m_viewCenter.y() - y_relative * m_pixelsPerUnit;
    return QPointF(widget_x, widget_y);
}

void ProjectionPlotWidget::drawAxes(QPainter& painter)
{
    painter.setPen(QPen(Qt::black, 1.5));
    // 축은 위젯의 물리적 중심(Centroid)에 그려집니다.
    painter.drawLine(0, m_viewCenter.y(), this->width(), m_viewCenter.y());
    painter.drawLine(m_viewCenter.x(), 0, m_viewCenter.x(), this->height());

    // ✨ [수정] 라벨을 "Aligned PC..."로 변경하여 재정의된 축임을 명시
    painter.drawText(m_viewCenter.x() + 5, 15, "Aligned PC2 (Green)");
    painter.drawText(this->width() - 120, m_viewCenter.y() - 5, "Aligned PC1 (Red)");
}

void ProjectionPlotWidget::drawPoints(QPainter& painter)
{
    painter.setBrush(Qt::blue);
    painter.setPen(Qt::NoPen);
    for(const auto& data : m_projectedPoints) {
        painter.drawEllipse(dataToWidget(data), 2, 2);
    }
}

void ProjectionPlotWidget::drawOutline(QPainter& painter)
{
    if (m_outlinePolygon.isEmpty()) return;

    QPainterPath path;
    path.moveTo(dataToWidget(m_outlinePolygon[0]));
    for(int i = 1; i < m_outlinePolygon.size(); ++i) {
        path.lineTo(dataToWidget(m_outlinePolygon[i]));
    }
    path.closeSubpath();

    painter.setPen(QPen(Qt::red, 2.0, Qt::SolidLine));
    painter.setBrush(Qt::NoBrush);
    painter.drawPath(path);
}

void ProjectionPlotWidget::drawCenterPoint(QPainter &painter)
{
    // 빨간 원의 위치를 외곽선 중심(BBox Center)으로 지정
    QPointF center_widget = dataToWidget(m_bboxCenterForRedDot);

    // 빨간색 원 (채우기)
    painter.setBrush(Qt::red);
    painter.setPen(Qt::NoPen);
    painter.drawEllipse(center_widget, 6, 6);

    // 흰색 테두리
    painter.setPen(QPen(Qt::white, 1.0));
    painter.setBrush(Qt::NoBrush);
    painter.drawEllipse(center_widget, 6, 6);
}


// 2D 포인트 간의 외적 (회전 방향 판별용)
qreal ProjectionPlotWidget::cross_product(const QPointF& o, const QPointF& a, const QPointF& b)
{
    return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
}

// Monotone Chain 알고리즘을 사용한 Convex Hull 계산
void ProjectionPlotWidget::calculateConvexHull()
{
    int n = m_projectedPoints.size();
    if (n < 3) return;

    QVector<QPointF> points = m_projectedPoints;
    std::sort(points.begin(), points.end(), [](const QPointF& a, const QPointF& b) {
        return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
    });

    QVector<QPointF> hull;

    // 아래쪽 외곽선
    for (int i = 0; i < n; ++i) {
        while (hull.size() >= 2 && cross_product(hull[hull.size()-2], hull.last(), points[i]) <= 0) {
            hull.pop_back();
        }
        hull.push_back(points[i]);
    }

    // 위쪽 외곽선
    int lower_hull_size = hull.size();
    for (int i = n - 2; i >= 0; --i) {
        while (hull.size() > lower_hull_size && cross_product(hull[hull.size()-2], hull.last(), points[i]) <= 0) {
            hull.pop_back();
        }
        hull.push_back(points[i]);
    }

    hull.pop_back(); // 시작점 중복 제거
    m_outlinePolygon = hull;
}
