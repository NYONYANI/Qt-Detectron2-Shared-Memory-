#include "xyplotwidget.h"
#include <QDebug>

XYPlotWidget::XYPlotWidget(QWidget *parent)
    : QWidget(parent)
    , m_hasCircle(false)
    , m_hasLine(false)
    , m_hasPerpLines(false) // ✨ [추가] 멤버 변수 초기화
{
    QPalette pal = palette();
    pal.setColor(QPalette::Window, Qt::white);
    setAutoFillBackground(true);
    setPalette(pal);
    resize(500, 500);

    m_zoom = 1.0f;
    m_panOffset = QPointF(0.0, 0.0);
    setFocusPolicy(Qt::StrongFocus);
}

void XYPlotWidget::updateData(const QVector<PlotData>& bodyData, const QVector<PlotData>& handleData)
{
    m_bodyPlotData = bodyData;
    m_handlePlotData = handleData;
    m_hasCircle = false;
    m_hasLine = false;
    m_hasPerpLines = false; // ✨ [추가] 상태 초기화

    // 1. 몸통 포인트로 원 피팅
    QVector<QPointF> bodyPoints;
    for(const auto& data : m_bodyPlotData) {
        bodyPoints.append(data.point);
    }
    if (bodyPoints.size() > 2) {
        m_fittedCircle = CircleFitter::fitCircleLeastSquares(bodyPoints);
        if (m_fittedCircle.radius > 0 && m_fittedCircle.radius < m_range) {
            m_hasCircle = true;
        }
    }

    // 2. 원 피팅 성공 시, 손잡이 데이터로 직선 피팅
    if (m_hasCircle && !m_handlePlotData.isEmpty()) {
        double sum_x = 0, sum_y = 0;
        for (const auto& data : m_handlePlotData) {
            sum_x += data.point.x();
            sum_y += data.point.y();
        }
        QPointF handleCentroid(sum_x / m_handlePlotData.size(), sum_y / m_handlePlotData.size());
        QPointF circleCenter(m_fittedCircle.centerX, m_fittedCircle.centerY);

        m_fittedLine.setPoints(circleCenter, handleCentroid);
        m_fittedLine.setLength(m_fittedLine.length() + m_fittedCircle.radius * 1.5);
        m_hasLine = true;

        // 3. ✨ [추가] 수직선 계산
        qreal originalAngle = m_fittedLine.angle(); // 손잡이 직선의 각도
        qreal perpAngle1 = originalAngle + 90.0;  // 반시계 방향 90도
        qreal perpAngle2 = originalAngle - 90.0;  // 시계 방향 90도

        // 첫 번째 수직선 (원의 중심에서 시작, 반지름 길이만큼)
        m_perpLine1.setP1(circleCenter);
        m_perpLine1.setAngle(perpAngle1);
        m_perpLine1.setLength(m_fittedCircle.radius);

        // 두 번째 수직선
        m_perpLine2.setP1(circleCenter);
        m_perpLine2.setAngle(perpAngle2);
        m_perpLine2.setLength(m_fittedCircle.radius);

        m_hasPerpLines = true;
    }

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
    drawFittedCircle(painter);
    drawFittedLine(painter);
    drawPerpendicularLines(painter); // ✨ [추가] 수직선 그리기 함수 호출
}

void XYPlotWidget::mousePressEvent(QMouseEvent *event) { m_lastPos = event->pos(); }
void XYPlotWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (event->buttons() & Qt::RightButton) {
        QPoint delta = event->pos() - m_lastPos;
        m_panOffset += delta;
        m_lastPos = event->pos();
        update();
    }
}
void XYPlotWidget::wheelEvent(QWheelEvent *event)
{
    if (event->angleDelta().y() > 0) m_zoom *= 1.1f;
    else m_zoom *= 1.0f / 1.1f;
    update();
}

void XYPlotWidget::drawGrid(QPainter& painter)
{
    painter.setPen(QPen(QColor(220, 220, 220), 1.0, Qt::DotLine));
    int width = this->width();
    int height = this->height();
    QPointF center = QPointF(width / 2.0, height / 2.0) + m_panOffset;
    float pixels_per_meter = qMin(width, height) / (2.0f * m_range) * m_zoom;
    float grid_step_meters = 0.1f;
    int grid_step_pixels = static_cast<int>(grid_step_meters * pixels_per_meter);
    if (grid_step_pixels <= 5) return;
    for (float x = center.x(); x < width; x += grid_step_pixels) painter.drawLine(x, 0, x, height);
    for (float x = center.x() - grid_step_pixels; x > 0; x -= grid_step_pixels) painter.drawLine(x, 0, x, height);
    for (float y = center.y(); y < height; y += grid_step_pixels) painter.drawLine(0, y, width, y);
    for (float y = center.y() - grid_step_pixels; y > 0; y -= grid_step_pixels) painter.drawLine(0, y, width, y);
}

void XYPlotWidget::drawAxes(QPainter& painter)
{
    painter.setPen(QPen(Qt::black, 1.5));
    int width = this->width();
    int height = this->height();
    QPointF center = QPointF(width / 2.0, height / 2.0) + m_panOffset;
    painter.drawLine(0, center.y(), width, center.y());
    painter.drawText(width - 80, center.y() - 5, "+X (Robot Base)");
    painter.drawLine(center.x(), 0, center.x(), height);
    painter.drawText(center.x() + 5, 15, "+Y (Robot Base)");
    painter.drawText(center.x() + 5, center.y() + 15, "Origin");
}

void XYPlotWidget::drawPoints(QPainter& painter)
{
    int width = this->width();
    int height = this->height();
    QPointF center = QPointF(width / 2.0, height / 2.0) + m_panOffset;
    float pixels_per_meter = qMin(width, height) / (2.0f * m_range) * m_zoom;

    for(const auto& data : m_bodyPlotData)
    {
        float widget_x = center.x() + data.point.x() * pixels_per_meter;
        float widget_y = center.y() - data.point.y() * pixels_per_meter;
        painter.setBrush(data.color);
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(QPointF(widget_x, widget_y), 3, 3);
    }
    for(const auto& data : m_handlePlotData)
    {
        float widget_x = center.x() + data.point.x() * pixels_per_meter;
        float widget_y = center.y() - data.point.y() * pixels_per_meter;
        painter.setBrush(data.color);
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(QPointF(widget_x, widget_y), 3, 3);
    }
}

void XYPlotWidget::drawFittedCircle(QPainter &painter)
{
    if (!m_hasCircle) return;
    int width = this->width();
    int height = this->height();
    QPointF origin = QPointF(width / 2.0, height / 2.0) + m_panOffset;
    float pixels_per_meter = qMin(width, height) / (2.0f * m_range) * m_zoom;
    float cx_widget = origin.x() + m_fittedCircle.centerX * pixels_per_meter;
    float cy_widget = origin.y() - m_fittedCircle.centerY * pixels_per_meter;
    float radius_widget = m_fittedCircle.radius * pixels_per_meter;
    painter.setBrush(Qt::NoBrush);
    painter.setPen(QPen(Qt::red, 2.0, Qt::SolidLine));
    painter.drawEllipse(QPointF(cx_widget, cy_widget), radius_widget, radius_widget);
    painter.setBrush(Qt::red);
    painter.setPen(Qt::NoPen);
    painter.drawEllipse(QPointF(cx_widget, cy_widget), 4, 4);
}

void XYPlotWidget::drawFittedLine(QPainter &painter)
{
    if (!m_hasLine) return;
    int width = this->width();
    int height = this->height();
    QPointF origin = QPointF(width / 2.0, height / 2.0) + m_panOffset;
    float pixels_per_meter = qMin(width, height) / (2.0f * m_range) * m_zoom;
    QPointF p1_widget(origin.x() + m_fittedLine.p1().x() * pixels_per_meter, origin.y() - m_fittedLine.p1().y() * pixels_per_meter);
    QPointF p2_widget(origin.x() + m_fittedLine.p2().x() * pixels_per_meter, origin.y() - m_fittedLine.p2().y() * pixels_per_meter);
    painter.setPen(QPen(Qt::blue, 2.0, Qt::SolidLine));
    painter.drawLine(p1_widget, p2_widget);
}

// ✨ [추가] 수직선을 그리는 함수 구현
void XYPlotWidget::drawPerpendicularLines(QPainter &painter)
{
    if (!m_hasPerpLines) return;

    int width = this->width();
    int height = this->height();
    QPointF origin = QPointF(width / 2.0, height / 2.0) + m_panOffset;
    float pixels_per_meter = qMin(width, height) / (2.0f * m_range) * m_zoom;

    // 점선 스타일 설정
    painter.setPen(QPen(Qt::darkGray, 1.5, Qt::DotLine));

    // 첫 번째 수직선 그리기
    QPointF p1_1_widget(origin.x() + m_perpLine1.p1().x() * pixels_per_meter, origin.y() - m_perpLine1.p1().y() * pixels_per_meter);
    QPointF p1_2_widget(origin.x() + m_perpLine1.p2().x() * pixels_per_meter, origin.y() - m_perpLine1.p2().y() * pixels_per_meter);
    painter.drawLine(p1_1_widget, p1_2_widget);

    // 두 번째 수직선 그리기
    QPointF p2_1_widget(origin.x() + m_perpLine2.p1().x() * pixels_per_meter, origin.y() - m_perpLine2.p1().y() * pixels_per_meter);
    QPointF p2_2_widget(origin.x() + m_perpLine2.p2().x() * pixels_per_meter, origin.y() - m_perpLine2.p2().y() * pixels_per_meter);
    painter.drawLine(p2_1_widget, p2_2_widget);
}
