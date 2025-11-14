#ifndef CIRCLEFITTER_H
#define CIRCLEFITTER_H

#include <QVector>
#include <QPointF>
#include <cmath>

// 최소자승법으로 계산된 원의 결과를 담을 구조체
struct CircleResult {
    float centerX = 0.0f;
    float centerY = 0.0f;
    float radius = 0.0f;
};

// 피팅 알고리즘을 담고 있는 정적 클래스
class CircleFitter {
public:
    // 2D 포인트 벡터를 받아 CircleResult를 반환하는 함수
    static CircleResult fitCircleLeastSquares(const QVector<QPointF>& points) {
        CircleResult result;
        if (points.size() < 3) {
            return result; // 최소 3개의 점이 필요
        }

        double sum_x = 0, sum_y = 0;
        double sum_xx = 0, sum_yy = 0, sum_xy = 0;
        double sum_xxx = 0, sum_yyy = 0, sum_xyy = 0, sum_yxx = 0;

        for (const auto& p : points) {
            double x = p.x();
            double y = p.y();
            double xx = x * x;
            double yy = y * y;
            sum_x += x;
            sum_y += y;
            sum_xx += xx;
            sum_yy += yy;
            sum_xy += x * y;
            sum_xxx += xx * x;
            sum_yyy += yy * y;
            sum_xyy += x * yy;
            sum_yxx += y * xx;
        }

        double N = points.size();
        double A = N * sum_xx - sum_x * sum_x;
        double B = N * sum_xy - sum_x * sum_y;
        double C = N * sum_yy - sum_y * sum_y;
        double D = 0.5 * (N * sum_xyy - sum_x * sum_yy + N * sum_xxx - sum_x * sum_xx);
        double E = 0.5 * (N * sum_yxx - sum_y * sum_xx + N * sum_yyy - sum_y * sum_yy);

        double det = A * C - B * B;
        if (std::abs(det) < 1e-10) {
            return result; // 행렬이 특이(singular)하면 계산 불가
        }

        double centerX = (D * C - B * E) / det;
        double centerY = (A * E - B * D) / det;
        double c = (sum_xx + sum_yy - 2 * centerX * sum_x - 2 * centerY * sum_y) / N;
        double radius = std::sqrt(c + centerX * centerX + centerY * centerY);

        result.centerX = static_cast<float>(centerX);
        result.centerY = static_cast<float>(centerY);
        result.radius = static_cast<float>(radius);

        return result;
    }
};

#endif // CIRCLEFITTER_H
