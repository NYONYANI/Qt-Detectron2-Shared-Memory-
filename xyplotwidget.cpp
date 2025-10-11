#include "xyplotwidget.h"
#include <QDebug>

XYPlotWidget::XYPlotWidget(QWidget *parent)
    : QWidget(parent)
{
    // ✨ [수정] 배경을 흰색으로 변경
    QPalette pal = palette();
    pal.setColor(QPalette::Window, Qt::white);
    setAutoFillBackground(true);
    setPalette(pal);
    resize(500, 500);

    // ✨ [추가] 확대/축소 및 이동 변수 초기화
    m_zoom = 1.0f;
    m_panOffset = QPointF(0.0, 0.0);
    setFocusPolicy(Qt::StrongFocus); // 마우스/키보드 이벤트 수신 설정
}

void XYPlotWidget::updateData(const QVector<PlotData>& data)
{
    m_plotData = data;
    update();
}

void XYPlotWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    drawGrid(painter);
    drawAxes(painter);
    drawPoints(painter);
}

// ✨ [추가] 마우스/휠 이벤트 핸들러 구현
void XYPlotWidget::mousePressEvent(QMouseEvent *event)
{
    m_lastPos = event->pos();
}

void XYPlotWidget::mouseMoveEvent(QMouseEvent *event)
{
    // 우클릭으로 이동
    if (event->buttons() & Qt::RightButton) {
        QPoint delta = event->pos() - m_lastPos;
        m_panOffset += delta;
        m_lastPos = event->pos();
        update();
    }
}

void XYPlotWidget::wheelEvent(QWheelEvent *event)
{
    if (event->angleDelta().y() > 0) {
        m_zoom *= 1.1f; // 확대
    } else {
        m_zoom *= 1.0f / 1.1f; // 축소
    }
    update();
}


void XYPlotWidget::drawGrid(QPainter& painter)
{
    // ✨ [수정] 줌과 패닝을 그리드에 적용
    painter.setPen(QPen(QColor(220, 220, 220), 1.0, Qt::DotLine)); // 밝은 회색 그리드
    int width = this->width();
    int height = this->height();
    QPointF center = QPointF(width / 2.0, height / 2.0) + m_panOffset;
    float pixels_per_meter = qMin(width, height) / (2.0f * m_range) * m_zoom;
    float grid_step_meters = 0.1f; // 10cm 간격
    int grid_step_pixels = static_cast<int>(grid_step_meters * pixels_per_meter);

    if (grid_step_pixels <= 5) return; // 그리드가 너무 촘촘해지면 그리지 않음

    // 세로선
    for (float x = center.x(); x < width; x += grid_step_pixels) painter.drawLine(x, 0, x, height);
    for (float x = center.x() - grid_step_pixels; x > 0; x -= grid_step_pixels) painter.drawLine(x, 0, x, height);
    // 가로선
    for (float y = center.y(); y < height; y += grid_step_pixels) painter.drawLine(0, y, width, y);
    for (float y = center.y() - grid_step_pixels; y > 0; y -= grid_step_pixels) painter.drawLine(0, y, width, y);
}

void XYPlotWidget::drawAxes(QPainter& painter)
{
    // ✨ [수정] 줌과 패닝을 축에 적용
    painter.setPen(QPen(Qt::black, 1.5)); // 검은색 축
    int width = this->width();
    int height = this->height();
    QPointF center = QPointF(width / 2.0, height / 2.0) + m_panOffset;

    // X축
    painter.drawLine(0, center.y(), width, center.y());
    painter.drawText(width - 80, center.y() - 5, "+X (Robot Base)");

    // Y축
    painter.drawLine(center.x(), 0, center.x(), height);
    painter.drawText(center.x() + 5, 15, "+Y (Robot Base)");

    painter.drawText(center.x() + 5, center.y() + 15, "Origin");
}

void XYPlotWidget::drawPoints(QPainter& painter)
{
    // ✨ [수정] 줌과 패닝을 포인트에 적용
    int width = this->width();
    int height = this->height();
    QPointF center = QPointF(width / 2.0, height / 2.0) + m_panOffset;
    float pixels_per_meter = qMin(width, height) / (2.0f * m_range) * m_zoom;

    for(const auto& data : m_plotData)
    {
        float widget_x = center.x() + data.point.x() * pixels_per_meter;
        float widget_y = center.y() - data.point.y() * pixels_per_meter;

        painter.setBrush(data.color);
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(QPointF(widget_x, widget_y), 5, 5);
    }
}
