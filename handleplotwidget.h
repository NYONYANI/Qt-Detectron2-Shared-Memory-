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

    // ✨ [수정됨] 중심선 계산: 영역 내 포인트들의 X, Y 중앙값 사용
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

            // ✨ [개선] 영역 내 포인트들의 X, Y 좌표를 각각 수집
            QVector<double> xValues, yValues;
            double minDistProj = std::numeric_limits<double>::max();
            double maxDistProj = std::numeric_limits<double>::lowest();
            QPointF currentMinPoint = curvePoint;
            QPointF currentMaxPoint = curvePoint;

            // 직사각형 영역 내 모든 점을 순회
            for (const QPointF& dataPoint : m_plotData)
            {
                if (data.rect.containsPoint(dataPoint, Qt::OddEvenFill)) {
                    // 중앙값 계산용 X, Y 값 수집
                    xValues.append(dataPoint.x());
                    yValues.append(dataPoint.y());

                    // 시각화용 min/max 점도 계속 추적
                    QVector2D diffVec(dataPoint - curvePoint);
                    double distProj = QVector2D::dotProduct(diffVec, normal);

                    if (distProj < minDistProj) {
                        minDistProj = distProj;
                        currentMinPoint = dataPoint;
                    }
                    if (distProj > maxDistProj) {
                        maxDistProj = distProj;
                        currentMaxPoint = dataPoint;
                    }
                }
            }

            // ✨ [수정된 로직] 영역 내 포인트가 있으면 X, Y 각각의 중앙값 사용
            if (!xValues.isEmpty())
            {
                // X, Y 각각 정렬하여 중앙값 찾기
                std::sort(xValues.begin(), xValues.end());
                std::sort(yValues.begin(), yValues.end());

                int midIdx = xValues.size() / 2;
                double medianX = xValues[midIdx];
                double medianY = yValues[midIdx];

                data.minPoint = currentMinPoint;
                data.maxPoint = currentMaxPoint;
                data.midPoint = QPointF(medianX, medianY);  // ✨ X, Y 각각의 중앙값
                data.valid = true;
            }
            else
            {
                // 사각형 안에 점이 아예 없으면 곡선 점을 중심선으로 사용
                data.minPoint = curvePoint;
                data.maxPoint = curvePoint;
                data.midPoint = curvePoint;
                data.valid = true;  // 끝단도 연결되도록
            }
            m_centerlineData.push_back(data);
        }
    }

    // 중심선 포인트 스무딩 (갭 채우기 및 양끝단 연결 로직)
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

        // --- 1단계: 유효한 포인트만 먼저 스무딩 ---
        QVector<QPointF> tempSmoothed(n);
        QVector<bool> isValidSmoothed(n, false);
        int validPointsFound = 0;

        for (int i = 0; i < n; ++i)
        {
            int startIdx = qMax(0, i - halfWindow);
            int endIdx = qMin(n - 1, i + halfWindow);

            QPointF sum(0, 0);
            int count = 0;
            for (int k = startIdx; k <= endIdx; ++k) {
                // 원본 데이터(midPoint)가 유효한지 검사
                if (m_centerlineData[k].valid) {
                    sum += m_centerlineData[k].midPoint;
                    count++;
                }
            }

            if (count > 0) {
                tempSmoothed[i] = sum / count;
                isValidSmoothed[i] = true;
                validPointsFound++;
            }
            // count == 0 이면 isValidSmoothed[i]는 false로 유지
        }

        // --- 2단계: 갭(Gap) 채우기 ---

        // 유효한 스무딩 점이 하나도 없으면, 곡선 점(curvePoint)으로 대체
        if (validPointsFound == 0) {
            qWarning() << "[Smooth] No valid points found during smoothing.";
            for(int i = 0; i < n; ++i) { m_smoothedCenterlinePoints[i] = m_centerlineData[i].curvePoint; }
            return;
        }

        // 첫 번째 유효 스무딩 인덱스 찾기
        int firstValidIdx = -1;
        for(int i=0; i<n; ++i) if(isValidSmoothed[i]) { firstValidIdx = i; break; }

        // 마지막 유효 스무딩 인덱스 찾기
        int lastValidIdx = n - 1;
        for(int i=n-1; i>=0; --i) if(isValidSmoothed[i]) { lastValidIdx = i; break; }

        // 2-1: 시작 갭 채우기 (Extrapolation)
        // 0부터 firstValidIdx 직전까지 모든 점을 첫 유효점으로 설정
        for(int i = 0; i < firstValidIdx; ++i) {
            m_smoothedCenterlinePoints[i] = tempSmoothed[firstValidIdx];
        }

        // 2-2: 끝 갭 채우기 (Extrapolation)
        // lastValidIdx 다음부터 끝까지 모든 점을 마지막 유효점으로 설정
        for(int i = lastValidIdx + 1; i < n; ++i) {
            m_smoothedCenterlinePoints[i] = tempSmoothed[lastValidIdx];
        }

        // 2-3: 중간 갭 채우기 (Interpolation)
        for (int i = firstValidIdx; i <= lastValidIdx; ++i)
        {
            if (isValidSmoothed[i]) {
                // 유효한 점은 그대로 복사
                m_smoothedCenterlinePoints[i] = tempSmoothed[i];
            } else {
                // 갭 시작 (i는 유효하지 않음)
                int gapStartIdx = i;
                int prevValidIdx = i - 1; // i >= firstValidIdx이므로 prevValidIdx는 항상 유효함

                // 갭 끝(다음 유효점) 찾기
                int nextValidIdx = gapStartIdx + 1;
                while(nextValidIdx <= lastValidIdx && !isValidSmoothed[nextValidIdx]) {
                    nextValidIdx++;
                }

                // 갭의 양 끝점 (p1은 이미 m_smoothedCenterlinePoints에 채워져 있음)
                QPointF p1 = m_smoothedCenterlinePoints[prevValidIdx];
                QPointF p2 = tempSmoothed[nextValidIdx]; // nextValidIdx는 유효함이 보장됨

                int gapLen = nextValidIdx - prevValidIdx; // (e.g., prev=3, next=7 -> 7-3 = 4)

                // 갭 내부를 선형 보간
                for(int k = gapStartIdx; k < nextValidIdx; ++k) {
                    // k가 prevValidIdx로부터 얼마나 떨어져 있는지 비율 계산
                    double t = static_cast<double>(k - prevValidIdx) / static_cast<double>(gapLen);
                    m_smoothedCenterlinePoints[k] = p1 * (1.0 - t) + p2 * t;
                }

                // 갭을 모두 채웠으므로, 다음 유효점 직전까지 인덱스 점프
                i = nextValidIdx - 1;
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
