#include "projectionplotwidget.h"
#include <QDebug>
#include <QPalette>
#include <algorithm> // std::sort
#include <QPainterPath>

ProjectionPlotWidget::ProjectionPlotWidget(QWidget *parent)
    : QWidget(parent)
{
    QPalette pal = palette();
    pal.setColor(QPalette::Window, Qt::white);
    setAutoFillBackground(true);
    setPalette(pal);
    resize(500, 500);
    setWindowTitle("PCA XY Projection Plot (Outline)");
}

void ProjectionPlotWidget::updateData(const QVector<QPointF>& projectedPoints)
{
    m_projectedPoints = projectedPoints;
    m_outlinePolygon.clear();

    if (m_projectedPoints.size() < 3) {
        qWarning() << "[ProjPlot] Not enough points for outline:" << m_projectedPoints.size();
        m_dataCenter = QPointF(0,0); // ✨ [추가] 포인트가 없으면 센터 초기화
    } else {
        // 데이터 범위 계산
        float minX = m_projectedPoints[0].x(), maxX = m_projectedPoints[0].x();
        float minY = m_projectedPoints[0].y(), maxY = m_projectedPoints[0].y();
        for (const QPointF& p : m_projectedPoints) {
            if (p.x() < minX) minX = p.x(); if (p.x() > maxX) maxX = p.x();
            if (p.y() < minY) minY = p.y(); if (p.y() > maxY) maxY = p.y();
        }
        // ✨ [수정] m_dataCenter는 PCA 평균(0,0)이 아닌, 2D BBox의 중심으로 설정
        m_dataCenter = QPointF((minX + maxX) / 2.0f, (minY + maxY) / 2.0f);
        float dataWidth = qMax(maxX - minX, 0.01f) * 1.1f;
        float dataHeight = qMax(maxY - minY, 0.01f) * 1.1f;
        m_range = qMax(dataWidth, dataHeight) / 2.0f;

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
    drawCenterPoint(painter); // ✨ [이 줄 추가]
}

QPointF ProjectionPlotWidget::dataToWidget(const QPointF& dataPoint)
{
    float x_relative = dataPoint.x() - m_dataCenter.x();
    float y_relative = dataPoint.y() - m_dataCenter.y();
    float widget_x = m_viewCenter.x() + x_relative * m_pixelsPerUnit;
    float widget_y = m_viewCenter.y() - y_relative * m_pixelsPerUnit;
    return QPointF(widget_x, widget_y);
}

void ProjectionPlotWidget::drawAxes(QPainter& painter)
{
    painter.setPen(QPen(Qt::black, 1.5));
    painter.drawLine(0, m_viewCenter.y(), this->width(), m_viewCenter.y());
    painter.drawLine(m_viewCenter.x(), 0, m_viewCenter.x(), this->height());
    painter.drawText(m_viewCenter.x() + 5, 15, "PC2 (Green)");
    painter.drawText(this->width() - 80, m_viewCenter.y() - 5, "PC1 (Red)");
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

// ✨ [아래 함수 새로 추가]
void ProjectionPlotWidget::drawCenterPoint(QPainter &painter)
{
    // m_dataCenter는 updateData에서 계산된 데이터 좌표계의 2D BBox 중심입니다.
    // 이 점은 dataToWidget 변환의 (0,0) 기준점이 되므로,
    // dataToWidget(m_dataCenter)는 항상 m_viewCenter (위젯의 픽셀 중심)와 같습니다.

    // 검은색 원으로 그립니다.
    painter.setBrush(Qt::black);
    painter.setPen(QPen(Qt::black, 2.0));
    painter.drawEllipse(m_viewCenter, 4, 4); // 반지름 4px
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
