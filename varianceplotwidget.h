#ifndef VARIANCEPLOTWIDGET_H
#define VARIANCEPLOTWIDGET_H

#include <QWidget>
#include <QVector>
#include <QPainter>
#include <cmath>

// 곡선 정보를 저장할 구조체
struct GaussianCurve {
    float sigma;
    QColor color;
    Qt::PenStyle style;
    int width;
    QString label;
};

class VariancePlotWidget : public QWidget {
    Q_OBJECT
public:
    explicit VariancePlotWidget(QWidget *parent = nullptr);

    // 기존 데이터 초기화
    void clear();

    // 곡선 추가 함수
    void addCurve(float sigma, QColor color, Qt::PenStyle style, int width, QString label);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    QVector<GaussianCurve> m_curves; // 곡선 리스트

    // 가우시안 Y값 계산 함수
    float getGaussianY(float x, float sigma);
};

#endif // VARIANCEPLOTWIDGET_H
