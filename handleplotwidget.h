#ifndef HANDLEPLOTWIDGET_H
#define HANDLEPLOTWIDGET_H

#include <QWidget>
#include <QVector>
#include <QPointF>
#include <QPainter>
#include <QPainterPath>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <limits>
#include <vector>
#include <tuple>
#include <QVector2D>
#include <QPolygonF>
#include <QRandomGenerator> // K-Means (이전 버전에서 사용되었을 수 있음)


class HandlePlotWidget : public QWidget
{
    Q_OBJECT

    enum class FitMode {
        Y_on_X, // y = f(x)
        X_on_Y  // x = f(y)
    };

    // 중심선 계산 관련 데이터를 묶어서 저장할 구조체
    struct CenterlineData {
        QPointF curvePoint;
        QVector2D normal;
        QVector2D tangent;
        QPointF minPoint;
        QPointF maxPoint;
        QPointF midPoint;
        QPolygonF rect;
        bool valid = false;
    };


public:
    explicit HandlePlotWidget(QWidget *parent = nullptr);
    void updateData(const QVector<QPointF>& points);
    QVector<QPointF> getSmoothedCenterlinePoints() const { return m_smoothedCenterlinePoints; };
    QVector<int> getSegmentIds() const { return m_segmentIds; }; // ✨ 이 함수가 public에 있는지 확인!

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    // 데이터 좌표 -> 위젯 좌표 변환
    QPointF dataToWidget(const QPointF& dataPoint);

    // 2차 다항식 피팅
    bool fitBestPolynomial();

    // 곡선 위의 점/벡터 계산
    std::tuple<QPointF, QVector2D, QVector2D> getPointVectorsOnCurve(double t);

    // 중심선 계산
    void calculateCenterlinePoints();
    void smoothCenterlinePoints(int windowSize = 3);

    // 그리기 함수들
    void drawPolynomialCurve(QPainter& painter, const QColor& color, double width);
    void drawNormalRects(QPainter& painter);
    void drawCenterline(QPainter& painter); // 분기 로직
    void drawSimpleCenterline(QPainter& painter); // 단색

    // 기울기 변화 분석 및 세그먼테이션 함수
    void analyzeAndSegmentBySlopeChange();


private:
    QVector<QPointF> m_plotData;
    QPointF m_dataCenter = QPointF(0, 0);
    float m_range = 0.1f;
    float m_minX = 0, m_maxX = 0, m_minY = 0, m_maxY = 0;
    QPointF m_viewCenter;
    float m_pixelsPerUnit = 1.0f;

    Eigen::Vector3d m_polyCoeffs;
    bool m_hasCurve;
    FitMode m_fitMode;

    std::vector<CenterlineData> m_centerlineData;
    QVector<QPointF> m_smoothedCenterlinePoints;

    // 세그먼테이션 결과 저장 (0, 1, 2)
    QVector<int> m_segmentIds;
    // 세그먼트 색상 (0: 녹색 \, 1: 파란색 _, 2: 빨간색 /)
    const static QColor m_segmentColors[3];
};

#endif // HANDLEPLOTWIDGET_H
