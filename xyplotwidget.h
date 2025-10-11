#ifndef XYPLOTWIDGET_H
#define XYPLOTWIDGET_H

#include <QWidget>
#include <QVector>
#include <QPointF>
#include <QColor>
#include <QPainter>

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

private:
    void drawGrid(QPainter& painter);
    void drawAxes(QPainter& painter);
    void drawPoints(QPainter& painter);

    QVector<PlotData> m_plotData;
    // 뷰포트 설정
    const float m_range = 1.0f; // 중심으로부터의 최대 거리 (미터)
};

#endif // XYPLOTWIDGET_H
