#ifndef XYPLOTWIDGET_H
#define XYPLOTWIDGET_H

#include <QWidget>
#include <QVector>
#include <QPointF>
#include <QColor>
#include <QPainter>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QLineF> // ✨ [추가] QLineF 클래스 포함
#include "circlefitter.h"

// 플롯할 개별 데이터 포인트를 위한 구조체
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
    // ✨ [수정] 몸통과 손잡이 데이터를 별도로 받는 함수로 변경
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
    void drawFittedLine(QPainter& painter); // ✨ [추가] 직선을 그리는 함수 선언

    QVector<PlotData> m_bodyPlotData;   // ✨ [수정] 변수 이름 변경 (body)
    QVector<PlotData> m_handlePlotData; // ✨ [추가] 손잡이 포인트 데이터 저장 변수
    const float m_range = 1.0f;

    float m_zoom;
    QPointF m_panOffset;
    QPoint m_lastPos;

    CircleResult m_fittedCircle;
    bool m_hasCircle;
    QLineF m_fittedLine; // ✨ [추가] 피팅된 직선 정보를 저장할 변수
    bool m_hasLine;
};

#endif // XYPLOTWIDGET_H
