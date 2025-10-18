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
#include <algorithm> // std::sort
#include <QDebug>
#include <vector> // std::vector
#include <cmath> // std::atan, std::cos, std::sin
#include <QVector2D>
#include <QPolygonF>
#include <tuple> // std::tuple 사용

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
    explicit HandlePlotWidget(QWidget *parent = nullptr)
        : QWidget(parent)
        , m_hasCurve(false)
        , m_fitMode(FitMode::Y_on_X)
    {
        QPalette pal = palette();
        pal.setColor(QPalette::Window, Qt::white);
        setAutoFillBackground(true);
        setPalette(pal);
        resize(500, 500);
        setWindowTitle("Handle Projection Plot (PCA + Rect Midpoint Centerline)");
    }

    void updateData(const QVector<QPointF>& points)
    {
        m_plotData = points;
        m_hasCurve = false;
        m_centerlineData.clear();
        m_smoothedCenterlinePoints.clear(); // 스무딩된 포인트 초기화
        // 최소 3개 점 (2차 다항식)
        if (m_plotData.size() < 3) {
            qWarning() << "[Fit] Not enough points for 2nd order polynomial fit:" << points.size();
            update();
            return;
        }

        // 데이터 범위 계산
        m_minX = m_maxX = m_plotData[0].x();
        m_minY = m_maxY = m_plotData[0].y();

        for (const QPointF& p : m_plotData) {
            if (p.x() < m_minX) m_minX = p.x();
            if (p.x() > m_maxX) m_maxX = p.x();
            if (p.y() < m_minY) m_minY = p.y();
            if (p.y() > m_maxY) m_maxY = p.y();
        }
        m_dataCenter = QPointF((m_minX + m_maxX) / 2.0f, (m_minY + m_maxY) / 2.0f);
        float dataWidth = m_maxX - m_minX;
        float dataHeight = m_maxY - m_minY;

        dataWidth = qMax(dataWidth, 0.01f) * 1.1f;
        dataHeight = qMax(dataHeight, 0.01f) * 1.1f;

        m_range = qMax(dataWidth, dataHeight) / 2.0f;

        // 2차 다항식 피팅
        if (fitBestPolynomial()) {
            // 중심선 포인트 계산 (직사각형 영역 + 양 끝점 중간 방식)
            calculateCenterlinePoints();
            // 중심선 스무딩 (끝점 처리 포함)
            smoothCenterlinePoints();
        }

        update();
    }

protected:
    void paintEvent(QPaintEvent *event) override
    {
        Q_UNUSED(event);
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        int width = this->width();
        int height = this->height();
        m_viewCenter = QPointF(width / 2.0, height / 2.0);
        m_pixelsPerUnit = qMin(width, height) / (2.0f * m_range);

        // 1. 축 그리기
        painter.setPen(QPen(Qt::black, 1.5));
        painter.drawLine(0, m_viewCenter.y(), width, m_viewCenter.y()); // X축 (PC1)
        painter.drawLine(m_viewCenter.x(), 0, m_viewCenter.x(), height); // Y축 (PC2)
        painter.drawText(m_viewCenter.x() + 5, 15, "PC2");
        painter.drawText(width - 40, m_viewCenter.y() - 5, "PC1");

        if (m_plotData.isEmpty()) return;

        // 2. 포인트 그리기
        painter.setBrush(Qt::blue);
        painter.setPen(Qt::NoPen);
        for(const auto& data : m_plotData)
        {
            QPointF widgetPos = dataToWidget(data);
            painter.drawEllipse(widgetPos, 3, 3);
        }

        // 3. 피팅된 2차 곡선 (더 진하게)
        drawPolynomialCurve(painter, QColor(255, 0, 0, 200), 1.5); // 진한 빨간색, 두께 1.5

        // 4. 직사각형 영역 그리기
        drawNormalRects(painter);

        // 5. 스무딩된 중심선 그리기
        drawCenterline(painter);
    }

private:
    // 데이터 좌표 -> 위젯 좌표 변환
    QPointF dataToWidget(const QPointF& dataPoint)
    {
        float x_relative = dataPoint.x() - m_dataCenter.x();
        float y_relative = dataPoint.y() - m_dataCenter.y();

        float widget_x = m_viewCenter.x() + x_relative * m_pixelsPerUnit;
        float widget_y = m_viewCenter.y() - y_relative * m_pixelsPerUnit; // Y축 뒤집기
        return QPointF(widget_x, widget_y);
    }

    // 2차 다항식 피팅 (두 방향 비교)
    bool fitBestPolynomial()
    {
        if (m_plotData.size() < 3) { // 최소 3점 필요
            m_hasCurve = false;
            return false;
        }

        int n = m_plotData.size();
        Eigen::MatrixXd X_Y(n, 3); // y = ax^2 + bx + c
        Eigen::VectorXd Y_Y(n);
        Eigen::MatrixXd X_X(n, 3); // x = ay^2 + by + c
        Eigen::VectorXd Y_X(n);

        for (int i = 0; i < n; ++i) {
            double x = m_plotData[i].x();
            double y = m_plotData[i].y();
            double x2 = x * x;
            double y2 = y * y;

            X_Y(i, 0) = x2; X_Y(i, 1) = x; X_Y(i, 2) = 1; Y_Y(i) = y;
            X_X(i, 0) = y2; X_X(i, 1) = y; X_X(i, 2) = 1; Y_X(i) = x;
        }

        Eigen::Vector3d coeffs_Y = X_Y.colPivHouseholderQr().solve(Y_Y);
        double error_Y = (Y_Y - X_Y * coeffs_Y).squaredNorm();
        Eigen::Vector3d coeffs_X = X_X.colPivHouseholderQr().solve(Y_X);
        double error_X = (Y_X - X_X * coeffs_X).squaredNorm();

        if (error_Y <= error_X) {
            m_polyCoeffs = coeffs_Y; m_fitMode = FitMode::Y_on_X;
        } else {
            m_polyCoeffs = coeffs_X; m_fitMode = FitMode::X_on_Y;
        }

        m_hasCurve = true;
        return true;
    }

    // 곡선 위의 점, 법선, 접선 벡터 계산
    std::tuple<QPointF, QVector2D, QVector2D> getPointVectorsOnCurve(double t)
    {
        double a = m_polyCoeffs(0);
        double b = m_polyCoeffs(1);
        double c = m_polyCoeffs(2);
        QPointF point;
        QVector2D tangent;
        QVector2D normal;

        if (m_fitMode == FitMode::Y_on_X) { // y = ax^2 + bx + c
            double x = m_minX + t * (m_maxX - m_minX);
            double y = a * x * x + b * x + c;
            point = QPointF(x, y);
            double dy_dx = 2 * a * x + b;
            tangent = QVector2D(1.0, dy_dx).normalized();
        } else { // x = ay^2 + by + c
            double y = m_minY + t * (m_maxY - m_minY);
            double x = a * y * y + b * y + c;
            point = QPointF(x, y);
            double dx_dy = 2 * a * y + b;
            tangent = QVector2D(dx_dy, 1.0).normalized();
        }
        normal = QVector2D(-tangent.y(), tangent.x()); // 법선 = 접선 90도 회전
        return {point, normal, tangent};
    }

    // 중심선 계산: 직사각형 영역 내 양 끝점의 중간점 사용
    void calculateCenterlinePoints()
    {
        if (!m_hasCurve) return;
        m_centerlineData.clear();

        int numSamples = 20;
        // 요청한 사각형 크기 값 사용
        const double rectHalfWidth = 0.01 / 2.0; // 폭 1cm의 절반
        const double rectHalfLength = (m_range / numSamples) * 0.6; // 샘플 간격의 60% 길이

        for (int i = 0; i <= numSamples; ++i)
        {
            CenterlineData data;
            double t = static_cast<double>(i) / numSamples;
            auto [curvePoint, normal, tangent] = getPointVectorsOnCurve(t);

            data.curvePoint = curvePoint;
            data.normal = normal;
            data.tangent = tangent;

            QPointF tanVec = tangent.toPointF();
            QPointF normVec = normal.toPointF();
            QPointF p1 = curvePoint + tanVec * rectHalfLength + normVec * rectHalfWidth;
            QPointF p2 = curvePoint - tanVec * rectHalfLength + normVec * rectHalfWidth;
            QPointF p3 = curvePoint - tanVec * rectHalfLength - normVec * rectHalfWidth;
            QPointF p4 = curvePoint + tanVec * rectHalfLength - normVec * rectHalfWidth;
            data.rect << p1 << p2 << p3 << p4;

            double minDistProj = std::numeric_limits<double>::max();
            double maxDistProj = -std::numeric_limits<double>::max();
            QPointF currentMinPoint = curvePoint;
            QPointF currentMaxPoint = curvePoint;
            bool minPointFound = false;
            bool maxPointFound = false;

            // 직사각형 영역 내 점들을 찾고, 그 중 법선 방향 양 끝점 찾기
            for (const QPointF& dataPoint : m_plotData)
            {
                if (data.rect.containsPoint(dataPoint, Qt::OddEvenFill)) {
                    QVector2D diffVec(dataPoint - curvePoint);
                    double distProj = QVector2D::dotProduct(diffVec, normal);

                    if (distProj < 0) { // 법선 반대 방향
                        if (!minPointFound || distProj < minDistProj) {
                            minDistProj = distProj;
                            currentMinPoint = dataPoint;
                            minPointFound = true;
                        }
                    } else { // 법선 방향
                        if (!maxPointFound || distProj > maxDistProj) {
                            maxDistProj = distProj;
                            currentMaxPoint = dataPoint;
                            maxPointFound = true;
                        }
                    }
                }
            }

            // 양 끝점을 모두 찾았으면 중간점 계산 및 저장
            if (minPointFound && maxPointFound)
            {
                data.minPoint = currentMinPoint; // 시각화용
                data.maxPoint = currentMaxPoint; // 시각화용
                data.midPoint = (data.minPoint + data.maxPoint) / 2.0; // 실제 양 끝점의 중간
                data.valid = true;
            } else {
                // 하나라도 못 찾으면 유효하지 않음
                data.midPoint = curvePoint; // 곡선 점 사용
                data.minPoint = curvePoint;
                data.maxPoint = curvePoint;
                data.valid = false;
            }
            m_centerlineData.push_back(data);
        }
    }

    // ✨ [수정] 중심선 포인트 스무딩 (끝점 처리 개선)
    void smoothCenterlinePoints(int windowSize = 3) // 홀수 사용 권장
    {
        m_smoothedCenterlinePoints.clear();
        if (m_centerlineData.empty()) return;

        int n = m_centerlineData.size();
        m_smoothedCenterlinePoints.resize(n); // 최종 크기 미리 할당

        if (n < windowSize) { // 데이터가 너무 적으면 원본 복사
            for(int i = 0; i < n; ++i) {
                m_smoothedCenterlinePoints[i] = m_centerlineData[i].midPoint;
            }
            return;
        }

        int halfWindow = windowSize / 2;

        // 첫 번째 유효한 midPoint 찾기
        int firstValidIdx = -1;
        for(int i=0; i<n; ++i) { if(m_centerlineData[i].valid) { firstValidIdx = i; break; } }

        // 마지막 유효한 midPoint 찾기
        int lastValidIdx = -1;
        for(int i=n-1; i>=0; --i) { if(m_centerlineData[i].valid) { lastValidIdx = i; break; } }

        // 유효한 점이 하나도 없으면 모든 스무딩 포인트를 curvePoint로 채움
        if (firstValidIdx == -1) {
            qWarning() << "[Smooth] No valid points found.";
            for(int i = 0; i < n; ++i) { m_smoothedCenterlinePoints[i] = m_centerlineData[i].curvePoint; }
            return;
        }
        // lastValidIdx가 유효하지 않거나 first보다 작으면 first 사용 (유효점 1개)
        if (lastValidIdx == -1 || lastValidIdx < firstValidIdx) lastValidIdx = firstValidIdx;

        // 스무딩 계산
        for (int i = 0; i < n; ++i)
        {
            // 스무딩 범위 [startIdx, endIdx] 계산 (끝점 처리 포함)
            int startIdx = qMax(0, i - halfWindow);
            int endIdx = qMin(n - 1, i + halfWindow);

            // 스무딩 범위 내 유효한 점들의 합과 개수 계산
            QPointF sum(0, 0);
            int count = 0;
            for (int k = startIdx; k <= endIdx; ++k) {
                if (m_centerlineData[k].valid) {
                    sum += m_centerlineData[k].midPoint;
                    count++;
                }
            }

            if (count > 0) {
                m_smoothedCenterlinePoints[i] = sum / count;
            } else {
                // 주변에 유효한 점이 없으면 가장 가까운 유효점 사용
                // (firstValidIdx 이전이면 firstValidIdx 사용, lastValidIdx 이후면 lastValidIdx 사용)
                if (i < firstValidIdx) {
                    m_smoothedCenterlinePoints[i] = m_centerlineData[firstValidIdx].midPoint;
                } else if (i > lastValidIdx) {
                    m_smoothedCenterlinePoints[i] = m_centerlineData[lastValidIdx].midPoint;
                } else {
                    // 중간인데 주변에 유효점 없으면 바로 이전 스무딩 점 사용 (선 끊김 방지)
                    if (i > 0) {
                        m_smoothedCenterlinePoints[i] = m_smoothedCenterlinePoints[i-1];
                    } else { // 맨 처음인데 주변에 유효점 없는 극단적 경우 (거의 발생 안 함)
                        m_smoothedCenterlinePoints[i] = m_centerlineData[i].curvePoint;
                    }
                }
            }
        }
    }


    // 2차 곡선 그리기 (옵션)
    void drawPolynomialCurve(QPainter& painter, const QColor& color, double width)
    {
        if (!m_hasCurve) return;

        QPainterPath path;
        int steps = 100;

        for (int i = 0; i <= steps; ++i) {
            double t = static_cast<double>(i) / steps;
            QPointF curvePoint = std::get<0>(getPointVectorsOnCurve(t));
            QPointF widgetPos = dataToWidget(curvePoint);
            if (i == 0) path.moveTo(widgetPos);
            else path.lineTo(widgetPos);
        }

        painter.setPen(QPen(color, width, Qt::SolidLine));
        painter.setBrush(Qt::NoBrush);
        painter.drawPath(path);
    }

    // 직사각형 영역 그리기
    void drawNormalRects(QPainter& painter)
    {
        if (m_centerlineData.empty()) return;

        // 반투명 회색 채우기, 테두리 없음
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor(128, 128, 128, 100)); // 반투명 회색

        for(const auto& data : m_centerlineData)
        {
            if (data.rect.size() == 4) // 유효한 사각형인지 확인
            {
                // 데이터 좌표계의 사각형 꼭짓점을 위젯 좌표계로 변환
                QPolygonF widgetRect;
                widgetRect << dataToWidget(data.rect[0])
                           << dataToWidget(data.rect[1])
                           << dataToWidget(data.rect[2])
                           << dataToWidget(data.rect[3]);
                painter.drawPolygon(widgetRect);

                // ✨ 옵션: 디버깅용으로 계산된 min/max 점 연결선 그리기 (Cyan색)
                // if(data.valid){
                //      painter.setPen(QPen(Qt::cyan, 1.0));
                //      painter.setBrush(Qt::NoBrush);
                //      painter.drawLine(dataToWidget(data.minPoint), dataToWidget(data.maxPoint));
                // }
            }
        }
    }


    // 스무딩된 중심선을 사용하여 그리기
    void drawCenterline(QPainter& painter)
    {
        if (m_smoothedCenterlinePoints.size() < 2) return;

        QPainterPath path;
        path.moveTo(dataToWidget(m_smoothedCenterlinePoints[0]));
        for(int i = 1; i < m_smoothedCenterlinePoints.size(); ++i) {
            path.lineTo(dataToWidget(m_smoothedCenterlinePoints[i]));
        }

        painter.setPen(QPen(Qt::magenta, 2.5, Qt::SolidLine));
        painter.setBrush(Qt::NoBrush);
        painter.drawPath(path);

        // 옵션: 스무딩된 중심점 자체도 그리기
        painter.setPen(Qt::NoPen);
        painter.setBrush(Qt::magenta);
        for(const auto& pt : m_smoothedCenterlinePoints) {
            painter.drawEllipse(dataToWidget(pt), 2, 2);
        }
    }


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
