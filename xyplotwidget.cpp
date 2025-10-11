#include "xyplotwidget.h"
#include <QDebug>

XYPlotWidget::XYPlotWidget(QWidget *parent)
    : QWidget(parent)
{
    // 위젯 배경색 설정
    QPalette pal = palette();
    pal.setColor(QPalette::Window, QColor(30, 30, 30));
    setAutoFillBackground(true);
    setPalette(pal);
}

// 외부에서 새로운 데이터를 받아와 위젯을 다시 그리도록 요청
void XYPlotWidget::updateData(const QVector<PlotData>& data)
{
    m_plotData = data;
    update(); // paintEvent()를 다시 호출
}

void XYPlotWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // 그리기 함수 호출
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

    // 월드 좌표를 위젯 좌표로 변환하는 스케일 계산
    float pixels_per_meter = qMin(width, height) / (2.0f * m_range);

    // 0.1 미터 간격으로 그리드 생성
    float grid_step_meters = 0.1f;
    int grid_step_pixels = static_cast<int>(grid_step_meters * pixels_per_meter);

    if (grid_step_pixels <= 0) return;

    // 세로선
    for (int x = center.x(); x < width; x += grid_step_pixels) {
        painter.drawLine(x, 0, x, height);
    }
    for (int x = center.x() - grid_step_pixels; x > 0; x -= grid_step_pixels) {
        painter.drawLine(x, 0, x, height);
    }

    // 가로선
    for (int y = center.y(); y < height; y += grid_step_pixels) {
        painter.drawLine(0, y, width, y);
    }
    for (int y = center.y() - grid_step_pixels; y > 0; y -= grid_step_pixels) {
        painter.drawLine(0, y, width, y);
    }
}

void XYPlotWidget::drawAxes(QPainter& painter)
{
    painter.setPen(QPen(Qt::white, 1.5));
    int width = this->width();
    int height = this->height();
    QPointF center(width / 2.0, height / 2.0);

    // X축 (가로)
    painter.drawLine(0, center.y(), width, center.y());
    painter.drawText(width - 20, center.y() - 5, "+X");

    // Z축 (세로, RealSense의 Z축이 화면의 Y축 역할)
    painter.drawLine(center.x(), 0, center.x(), height);
    painter.drawText(center.x() + 5, 15, "+Z (forward)");

    // 원점 표시
    painter.drawText(center.x() + 5, center.y() + 15, "Origin (Camera)");
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
        // 월드 좌표(m)를 위젯 픽셀 좌표로 변환
        // X는 그대로, Z는 위젯의 Y좌표가 되며, 방향을 맞추기 위해 부호 반전
        float widget_x = center.x() + data.point.x() * pixels_per_meter;
        float widget_y = center.y() - data.point.y() * pixels_per_meter;

        // 원 그리기
        painter.setBrush(data.color);
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(QPointF(widget_x, widget_y), 5, 5);

        // 라벨 표시
        painter.setPen(Qt::white);
        painter.drawText(widget_x + 8, widget_y + 4, QString("%1 (%2, %3)")
                         .arg(data.label)
                         .arg(data.point.x(), 0, 'f', 2)
                         .arg(data.point.y(), 0, 'f', 2));
    }
}
