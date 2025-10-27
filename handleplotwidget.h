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
#include <vector> // std::vector
#include <tuple> // std::tuple 사용
#include <QVector2D> // QVector2D, QPolygonF 등을 사용하므로 추가
#include <QPolygonF>


class HandlePlotWidget : public QWidget
{
    Q_OBJECT

    enum class FitMode {
        Y_on_X, // y = f(x)
        X_on_Y  // x = f(y)
    };

    // 중심선 계산 관련 데이터를 묶어서 저장할 구조체
    struct CenterlineData {
        QPointF curvePoint; // 곡선 위의 샘플 점
        QVector2D normal;   // 해당 점에서의 법선 벡터 (데이터 좌표계)
        QVector2D tangent;  // 해당 점에서의 접선 벡터 (데이터 좌표계)
        QPointF minPoint;   // (시각화용) 법선 반대 방향 끝 실제 데이터 점
        QPointF maxPoint;   // (시각화용) 법선 방향 끝 실제 데이터 점
        QPointF midPoint;   // 직사각형 영역 내 양 끝점의 중간 위치
        QPolygonF rect;     // 계산에 사용된 직사각형 영역 (데이터 좌표계)
        bool valid = false; // 이 데이터가 유효한지 (영역 내 min/max 점을 찾았는지)
    };


public:
    explicit HandlePlotWidget(QWidget *parent = nullptr);
    void updateData(const QVector<QPointF>& points);
    QVector<QPointF> getSmoothedCenterlinePoints() const { return m_smoothedCenterlinePoints; };
protected:
    void paintEvent(QPaintEvent *event) override;

private:
    // 데이터 좌표 -> 위젯 좌표 변환
    QPointF dataToWidget(const QPointF& dataPoint);

    // 2차 다항식 피팅 (두 방향 비교)
    bool fitBestPolynomial();

    // 곡선 위의 점, 법선, 접선 벡터 계산
    std::tuple<QPointF, QVector2D, QVector2D> getPointVectorsOnCurve(double t);

    // 중심선 계산: 영역 내 포인트들의 X, Y 중앙값 사용
    void calculateCenterlinePoints();

    // 중심선 포인트 스무딩 (갭 채우기 및 양끝단 연결 로직)
    void smoothCenterlinePoints(int windowSize = 3);

    // 2차 곡선 그리기 (옵션)
    void drawPolynomialCurve(QPainter& painter, const QColor& color, double width);

    // 직사각형 영역 그리기
    void drawNormalRects(QPainter& painter);

    // 스무딩된 중심선을 사용하여 그리기
    void drawCenterline(QPainter& painter);


private:
    QVector<QPointF> m_plotData;
    QPointF m_dataCenter = QPointF(0, 0);
    float m_range = 0.1f;
    float m_minX = 0, m_maxX = 0;
    float m_minY = 0, m_maxY = 0;

    QPointF m_viewCenter;
    float m_pixelsPerUnit = 1.0f;

    // 2차 다항식 계수 (a, b, c)
    Eigen::Vector3d m_polyCoeffs;
    bool m_hasCurve;
    FitMode m_fitMode;

    // 중심선 관련 데이터 저장 벡터 (원본 midPoint 포함)
    std::vector<CenterlineData> m_centerlineData;
    // 스무딩된 중심선 포인트 저장
    QVector<QPointF> m_smoothedCenterlinePoints;
};

#endif // HANDLEPLOTWIDGET_H
