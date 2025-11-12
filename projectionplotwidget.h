#ifndef PROJECTIONPLOTWIDGET_H
#define PROJECTIONPLOTWIDGET_H

#include <QWidget>
#include <QVector>
#include <QPointF>
#include <QPolygonF>
#include <QPainter>

class ProjectionPlotWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ProjectionPlotWidget(QWidget *parent = nullptr);

    // ✨ [추가] 2D 데이터 중심점(BBox)을 반환하는 getter
    QPointF getDataCenter() const { return m_dataCenter; }

public slots:
    void updateData(const QVector<QPointF>& projectedPoints);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    // Convex Hull (외곽선) 계산
    void calculateConvexHull();
    qreal cross_product(const QPointF& o, const QPointF& a, const QPointF& b);

    // 그리기 헬퍼
    QPointF dataToWidget(const QPointF& dataPoint);
    void drawAxes(QPainter& painter);
    void drawPoints(QPainter& painter);
    void drawOutline(QPainter& painter);
    void drawCenterPoint(QPainter& painter);

    QVector<QPointF> m_projectedPoints;
    QPolygonF m_outlinePolygon;

    // 뷰포트 관리
    QPointF m_dataCenter = QPointF(0, 0); // ✨ 이 값을 getter로 반환
    float m_range = 0.1f; // 기본 10cm
    QPointF m_viewCenter;
    float m_pixelsPerUnit = 1.0f;
};

#endif // PROJECTIONPLOTWIDGET_H
