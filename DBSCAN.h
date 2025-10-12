#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>
#include <cmath>
#include <QDebug>

// 3D 포인트를 위한 간단한 구조체
struct Point3D {
    float x, y, z;
    int clusterId; // 0: unclassified, -1: noise
    int originalIndex; // 원본 포인트 배열에서의 인덱스
};

class DBSCAN {
public:
    // eps: 이웃으로 간주할 최대 거리 (epsilon)
    // minPts: 군집을 형성하기 위한 최소 포인트 수
    // points: 군집화할 포인트들의 벡터
    DBSCAN(float eps, int minPts, std::vector<Point3D>& points)
        : epsilon(eps), minPoints(minPts), points(points) {}

    void run() {
        int clusterId = 1;
        for (auto& point : points) {
            if (point.clusterId == 0) { // 아직 방문하지 않았다면
                if (expandCluster(point, clusterId)) {
                    clusterId++;
                }
            }
        }
    }

private:
    float epsilon;
    int minPoints;
    std::vector<Point3D>& points;

    // 두 점 사이의 유클리드 거리 계산
    float calculateDistance(const Point3D& p1, const Point3D& p2) {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
    }

    // 주어진 점의 epsilon 반경 내에 있는 이웃 점들을 찾음
    std::vector<int> regionQuery(const Point3D& point) {
        std::vector<int> neighbors;
        for (size_t i = 0; i < points.size(); ++i) {
            if (calculateDistance(point, points[i]) < epsilon) {
                neighbors.push_back(i);
            }
        }
        return neighbors;
    }

    // 군집 확장
    bool expandCluster(Point3D& point, int clusterId) {
        std::vector<int> seeds = regionQuery(point);

        if (seeds.size() < minPoints) {
            point.clusterId = -1; // Noise로 표시
            return false;
        }

        // 핵심 포인트(Core point)이므로 새로운 군집 시작
        for (int seed_idx : seeds) {
            points[seed_idx].clusterId = clusterId;
        }
        seeds.erase(seeds.begin()); // 자기 자신은 제거

        // 이웃들을 순회하며 군집 확장
        while (!seeds.empty()) {
            Point3D& currentPoint = points[seeds.front()];
            std::vector<int> result = regionQuery(currentPoint);

            if (result.size() >= minPoints) {
                for (int result_idx : result) {
                    Point3D& resultPoint = points[result_idx];
                    if (resultPoint.clusterId == 0 || resultPoint.clusterId == -1) { // 방문 안했거나 노이즈였던 포인트
                        if (resultPoint.clusterId == 0) {
                             seeds.push_back(result_idx);
                        }
                        resultPoint.clusterId = clusterId;
                    }
                }
            }
            seeds.erase(seeds.begin());
        }
        return true;
    }
};

#endif // DBSCAN_H
