#include "varianceplotwidget.h"
#include <QDebug>
#include <QtMath>
#include <QPainterPath>

VariancePlotWidget::VariancePlotWidget(QWidget *parent) : QWidget(parent)
{
    resize(600, 400);
    setWindowTitle("PCA Variance Plot");
    QPalette pal = palette();
    pal.setColor(QPalette::Window, Qt::white);
    setAutoFillBackground(true);
    setPalette(pal);
}

void VariancePlotWidget::clear()
{
    m_curves.clear();
    update();
}

void VariancePlotWidget::addCurve(float sigma, QColor color, Qt::PenStyle style, int width, QString label)
{
    // 구조체 생성 후 리스트에 추가
    m_curves.append({sigma, color, style, width, label});
    update();
}

// 정규 분포 공식: f(x) = (1 / (σ * sqrt(2π))) * exp(-0.5 * (x/σ)^2)
float VariancePlotWidget::getGaussianY(float x, float sigma)
{
    if (sigma < 1e-6f) return 0.0f; // 0에 가까우면 계산 불가
    float variance = sigma * sigma;
    float exponent = -(x * x) / (2.0f * variance);
    return (1.0f / (sigma * std::sqrt(2.0f * M_PI))) * std::exp(exponent);
}

void VariancePlotWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    if (m_curves.isEmpty()) return;

    int w = width();
    int h = height();
    int margin = 50;
    int graphW = w - 2 * margin;
    int graphH = h - 2 * margin;

    // 1. X축 범위 결정 (등록된 곡선 중 가장 큰 분산 기준)
    float maxSigma = 0.0f;
    for (const auto& c : m_curves) {
        if (c.sigma > maxSigma) maxSigma = c.sigma;
    }
    if (maxSigma < 1e-6f) maxSigma = 0.01f; // 기본값

    // 3.5 시그마 범위까지 표시 (데이터의 약 99.9% 커버)
    float xRange = 3.5f * maxSigma;
    float minX = -xRange;
    float maxX = xRange;

    // 2. Y축 범위 결정 (가장 작은 분산이 가장 높은 피크를 가짐)
    float minSigma = 1e9f;
    for (const auto& c : m_curves) {
        if (c.sigma > 1e-6f && c.sigma < minSigma) minSigma = c.sigma;
    }
    // 너무 작은 값이면 maxSigma로 제한하여 그래프가 폭발하지 않게 함
    if (minSigma > 1e8f) minSigma = maxSigma;

    float maxY = getGaussianY(0, minSigma);
    if (maxY < 1e-6f) maxY = 1.0f;
    maxY *= 1.1f; // 상단 여백 10%

    // 좌표 변환 람다
    auto toScreen = [&](float x, float y) -> QPointF {
        float sx = margin + (x - minX) / (maxX - minX) * graphW;
        float sy = h - margin - (y / maxY) * graphH;
        return QPointF(sx, sy);
    };

    // 3. 축 그리기
    painter.setPen(QPen(Qt::black, 2));
    painter.drawLine(margin, h - margin, w - margin, h - margin); // X축
    painter.drawLine(w/2, h - margin, w/2, margin); // 중심선 (평균 = 0)

    painter.drawText(w - margin - 20, h - margin + 20, "+Range");
    painter.drawText(margin, h - margin + 20, "-Range");
    painter.drawText(w/2 - 15, h - margin + 20, "Mean");
    painter.drawText(margin, margin - 10, "Density");

    // 4. 곡선 그리기
    int steps = 200; // 해상도
    for (const auto& c : m_curves) {
        if (c.sigma < 1e-6f) continue;

        QPainterPath path;
        bool first = true;
        for (int i = 0; i <= steps; ++i) {
            float t = (float)i / steps;
            float x = minX + t * (maxX - minX);
            float y = getGaussianY(x, c.sigma);

            QPointF pt = toScreen(x, y);
            if (first) { path.moveTo(pt); first = false; }
            else { path.lineTo(pt); }
        }

        painter.setPen(QPen(c.color, c.width, c.style));
        painter.setBrush(Qt::NoBrush);
        painter.drawPath(path);
    }

    // 5. 범례 (Legend) 그리기
    int legX = w - 160;
    int legY = margin;
    int stepY = 20;

    painter.setFont(QFont("Arial", 9));
    // 배경 박스
    painter.setPen(Qt::black);
    painter.setBrush(QColor(255, 255, 255, 200));
    painter.drawRect(legX - 5, legY - 10, 160, m_curves.size() * stepY + 15);

    for (int i = 0; i < m_curves.size(); ++i) {
        const auto& c = m_curves[i];
        int y = legY + i * stepY;

        painter.setPen(QPen(c.color, 2, c.style));
        painter.drawLine(legX, y - 4, legX + 25, y - 4);

        painter.setPen(Qt::black);
        // 라벨과 표준편차 값(mm 단위) 표시
        QString label = c.label + QString(" (%1mm)").arg(c.sigma * 1000, 0, 'f', 1);
        painter.drawText(legX + 30, y, label);
    }
}
