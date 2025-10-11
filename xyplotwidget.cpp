#include "xyplotwidget.h"
#include <QDebug>

XYPlotWidget::XYPlotWidget(QWidget *parent)
    : QWidget(parent)
{
    QPalette pal = palette();
    pal.setColor(QPalette::Window, QColor(30, 30, 30));
    setAutoFillBackground(true);
    setPalette(pal);
    resize(500, 500); // 기본 크기 설정
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

    // ✨ [수정] 배경을 다시 칠하지 않도록 주석 처리 (생성자에서 설정)
    // painter.fillRect(rect(), QColor(30, 30, 30));

    drawGrid(painter);
    drawAxes(painter);
    drawPoints(painter);
}

void XYPlotWidget::drawGrid(QPainter& painter)
{
    painter.setPen(QPen(QColor(60, 60, 60), 1.0, Qt::DotLine));
    int width = this->width();
    int height = this->height();
    QPointF center(width / 2.0, height / 2.0);
    float pixels_per_meter = qMin(width, height) / (2.0f * m_range);
    float grid_step_meters = 0.1f; // 10cm 간격
    int grid_step_pixels = static_cast<int>(grid_step_meters * pixels_per_meter);
    if (grid_step_pixels <= 0) return;

    for (int x = center.x(); x < width; x += grid_step_pixels) painter.drawLine(x, 0, x, height);
    for (int x = center.x() - grid_step_pixels; x > 0; x -= grid_step_pixels) painter.drawLine(x, 0, x, height);
    for (int y = center.y(); y < height; y += grid_step_pixels) painter.drawLine(0, y, width, y);
    for (int y = center.y() - grid_step_pixels; y > 0; y -= grid_step_pixels) painter.drawLine(0, y, width, y);
}

void XYPlotWidget::drawAxes(QPainter& painter)
{
    painter.setPen(QPen(Qt::white, 1.5));
    int width = this->width();
    int height = this->height();
    QPointF center(width / 2.0, height / 2.0);

    painter.drawLine(0, center.y(), width, center.y());
    painter.drawText(width - 80, center.y() - 5, "+X (Robot Base)");

    painter.drawLine(center.x(), 0, center.x(), height);
    painter.drawText(center.x() + 5, 15, "+Y (Robot Base)");

    painter.drawText(center.x() + 5, center.y() + 15, "Origin (Robot Base)");
}

void XYPlotWidget::drawPoints(QPainter& painter)
{
    int width = this->width();
    int height = this->height();
    QPointF center(width / 2.0, height / 2.0);
    float pixels_per_meter = qMin(width, height) / (2.0f * m_range);
    painter.setFont(QFont("Monospace", 10));

    for(const auto& data : m_plotData)
    {
        float widget_x = center.x() + data.point.x() * pixels_per_meter;
        float widget_y = center.y() - data.point.y() * pixels_per_meter;

        painter.setBrush(data.color);
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(QPointF(widget_x, widget_y), 5, 5);

        painter.setPen(Qt::white);
        painter.drawText(widget_x + 8, widget_y + 4, QString("%1 (%2, %3)")
                         .arg(data.label)
                         .arg(data.point.x(), 0, 'f', 2)
                         .arg(data.point.y(), 0, 'f', 2));
    }
}
