#ifndef XYPLOTWIDGET_H
#define XYPLOTWIDGET_H

#include <QWidget>
#include <QVector>
#include <QPointF>
#include <QColor>
#include <QPainter>
#include <QMouseEvent>
#include <QWheelEvent>

// 플롯할 개별 데이터 포인트를 위한 구조체
struct PlotData {
    QPointF point; // x, z 좌표 (미터 단위)
    QColor color;
    QString label;
};

class XYPlotWidget : public QWidget
{
    Q_OBJECT

public:
    explicit XYPlotWidget(QWidget *parent = nullptr);
    void updateData(const QVector<PlotData>& data);

protected:
    void paintEvent(QPaintEvent *event) override;
    // ✨ [추가] 마우스/휠 이벤트 핸들러 선언
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;


private:
    void drawGrid(QPainter& painter);
    void drawAxes(QPainter& painter);
    void drawPoints(QPainter& painter);

    QVector<PlotData> m_plotData;
    const float m_range = 1.0f; // 중심으로부터의 최대 거리 (미터)

    // ✨ [추가] 확대/축소 및 이동을 위한 변수
    float m_zoom;
    QPointF m_panOffset;
    QPoint m_lastPos;
};

#endif // XYPLOTWIDGET_H
