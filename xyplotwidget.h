#ifndef XYPLOTWIDGET_H
#define XYPLOTWIDGET_H

#include <QWidget>
#include <QVector>
#include <QPointF>
#include <QColor>
#include <QPainter>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QLineF>
#include "circlefitter.h"

struct PlotData {
    QPointF point;
    QColor color;
    QString label;
};

class XYPlotWidget : public QWidget
{
    Q_OBJECT

public:
    explicit XYPlotWidget(QWidget *parent = nullptr);
    void updateData(const QVector<PlotData>& bodyData, const QVector<PlotData>& handleData);

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

private:
    void drawGrid(QPainter& painter);
    void drawAxes(QPainter& painter);
    void drawPoints(QPainter& painter);
    void drawFittedCircle(QPainter& painter);
    void drawFittedLine(QPainter& painter);
    void drawPerpendicularLines(QPainter& painter); // ✨ [추가] 수직선을 그리는 함수 선언

    QVector<PlotData> m_bodyPlotData;
    QVector<PlotData> m_handlePlotData;
    const float m_range = 1.0f;

    float m_zoom;
    QPointF m_panOffset;
    QPoint m_lastPos;

    CircleResult m_fittedCircle;
    bool m_hasCircle;
    QLineF m_fittedLine;
    bool m_hasLine;

    // ✨ [추가] 수직선 정보를 저장할 변수
    QLineF m_perpLine1;
    QLineF m_perpLine2;
    bool m_hasPerpLines;
};

#endif // XYPLOTWIDGET_H
