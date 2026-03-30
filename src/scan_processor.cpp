/*
 * Copyright (c) 2026 김승연
 *
 * This software is released under the MIT License.
 * See LICENSE file in the project root for details.
 *
 * Project: 데이터 무결성 보증형 디지털 트윈 관제 플랫폼
 * Module : EMBEDDED - LD19 LiDAR 센서 인터페이스 및 객체 탐지
 */

/**
 * @file  scan_processor.cpp
 * @brief 노이즈 필터 + 극좌표→직교좌표 변환 + DBSCAN 클러스터링 구현
 * @date  2026
 */

#include "scan_processor.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <numeric>

namespace ld19 {

// ── 생성자 ──────────────────────────────────────────────────────────
ScanProcessor::ScanProcessor(const FilterParams& filter, const DBSCANParams& dbscan)
    : filter_(filter), dbscan_(dbscan) {}

// ── 전체 파이프라인 ─────────────────────────────────────────────────
size_t ScanProcessor::Process(const ScanFrame& raw, std::vector<Cluster>& clusters) {
    // 1) 노이즈 필터
    ScanFrame filtered;
    FilterNoise(raw, filtered);

    if (filtered.empty()) {
        clusters.clear();
        return 0;
    }

    // 2) 극좌표 → 직교좌표
    std::vector<CartesianPoint> cart;
    ToCartesian(filtered, cart);

    // 3) DBSCAN 클러스터링
    ClusterDBSCAN(cart, clusters);

    // 4) 근접 클러스터 병합 (보행자 다리 분리 방지)
    if (filter_.merge_radius_mm > 0.0) {
        MergeClusters(clusters);
    }

    return filtered.size();
}

// ── 노이즈 필터 ─────────────────────────────────────────────────────
//   거리 0~30mm: 센서 근접 노이즈 제거
//   거리 12000mm 초과: LD19 최대 측정거리 초과 → 무효
void ScanProcessor::FilterNoise(const ScanFrame& raw, ScanFrame& filtered) {
    filtered.clear();
    filtered.reserve(raw.size());

    for (const auto& pt : raw) {
        if (pt.intensity < filter_.min_intensity)            continue;
        if (pt.distance_mm <= filter_.min_distance_mm ||
            pt.distance_mm >  filter_.max_distance_mm)       continue;
        if (pt.angle_deg < filter_.fov_min_deg ||
            pt.angle_deg > filter_.fov_max_deg)              continue;
        filtered.push_back(pt);
    }
}

// ── 극좌표 → 직교좌표 변환 ──────────────────────────────────────────
//   LiDAR 기준 좌표계:
//     X = distance * cos(angle)    (전방이 0°)
//     Y = distance * sin(angle)    (반시계 방향 +)
void ScanProcessor::ToCartesian(const ScanFrame& polar,
                                std::vector<CartesianPoint>& cart) {
    cart.clear();
    cart.reserve(polar.size());

    for (const auto& pt : polar) {
        double rad = pt.angle_deg * kDegToRad;
        cart.push_back(CartesianPoint{
            .x_mm        = pt.distance_mm * std::cos(rad),
            .y_mm        = pt.distance_mm * std::sin(rad),
            .angle_deg   = pt.angle_deg,
            .distance_mm = pt.distance_mm,
            .intensity   = pt.intensity,
        });
    }
}

// ── 그리드 인덱스 구축 ──────────────────────────────────────────────
void ScanProcessor::BuildGrid(const std::vector<CartesianPoint>& points,
                              double cell_size, GridIndex& grid) {
    grid.clear();
    for (size_t i = 0; i < points.size(); ++i) {
        GridKey key{
            static_cast<int32_t>(std::floor(points[i].x_mm / cell_size)),
            static_cast<int32_t>(std::floor(points[i].y_mm / cell_size))
        };
        grid[key].push_back(i);
    }
}

// ── RangeQueryGrid: 그리드 가속 eps 반경 이웃 검색 ──────────────────
//   cell_size = epsilon 이므로 자신 + 주변 8셀만 탐색 → O(1) 평균
void ScanProcessor::RangeQueryGrid(
    const std::vector<CartesianPoint>& points,
    const GridIndex& grid,
    double cell_size,
    size_t idx,
    double eps_sq,
    std::vector<size_t>& neighbors)
{
    neighbors.clear();
    const double px = points[idx].x_mm;
    const double py = points[idx].y_mm;
    const int32_t gx = static_cast<int32_t>(std::floor(px / cell_size));
    const int32_t gy = static_cast<int32_t>(std::floor(py / cell_size));

    for (int32_t dx = -1; dx <= 1; ++dx) {
        for (int32_t dy = -1; dy <= 1; ++dy) {
            auto it = grid.find(GridKey{gx + dx, gy + dy});
            if (it == grid.end()) continue;

            for (size_t i : it->second) {
                double ddx = points[i].x_mm - px;
                double ddy = points[i].y_mm - py;
                if (ddx * ddx + ddy * ddy <= eps_sq) {
                    neighbors.push_back(i);
                }
            }
        }
    }
}

// ── DBSCAN 클러스터링 ───────────────────────────────────────────────
//   epsilon  = 150mm  (이웃 탐색 반경)
//   minPts   = 5      (코어 포인트 조건)
//
//   알고리즘:
//   1. 모든 포인트를 UNVISITED로 초기화
//   2. 각 포인트에 대해 eps 반경 내 이웃 검색
//   3. 이웃 수 >= minPts → 새 클러스터 생성, 이웃 확장
//   4. 이웃 수 < minPts → NOISE로 마킹 (나중에 다른 클러스터에 편입 가능)
void ScanProcessor::ClusterDBSCAN(const std::vector<CartesianPoint>& points,
                                  std::vector<Cluster>& clusters) {
    clusters.clear();

    const size_t n = points.size();
    if (n == 0) return;

    const double eps_sq = dbscan_.epsilon_mm * dbscan_.epsilon_mm;
    const double cell_size = dbscan_.epsilon_mm;  // 그리드 셀 크기 = epsilon

    // 그리드 인덱스 구축 (O(n))
    GridIndex grid;
    BuildGrid(points, cell_size, grid);

    // 레이블: UNVISITED(-1), NOISE(-2), 또는 클러스터 ID (0, 1, 2, ...)
    std::vector<int> labels(n, UNVISITED);
    int cluster_id = 0;

    std::vector<size_t> neighbors;
    std::vector<size_t> sub_neighbors;

    for (size_t i = 0; i < n; ++i) {
        if (labels[i] != UNVISITED) continue;

        RangeQueryGrid(points, grid, cell_size, i, eps_sq, neighbors);

        if (neighbors.size() < dbscan_.min_points) {
            labels[i] = NOISE;
            continue;
        }

        int cid = cluster_id++;
        labels[i] = cid;

        for (size_t q = 0; q < neighbors.size(); ++q) {
            size_t nb = neighbors[q];

            if (labels[nb] == NOISE) {
                labels[nb] = cid;
                continue;
            }
            if (labels[nb] != UNVISITED) continue;

            labels[nb] = cid;

            RangeQueryGrid(points, grid, cell_size, nb, eps_sq, sub_neighbors);

            if (sub_neighbors.size() >= dbscan_.min_points) {
                for (size_t sn : sub_neighbors) {
                    if (labels[sn] == UNVISITED || labels[sn] == NOISE) {
                        neighbors.push_back(sn);
                    }
                }
            }
        }
    }

    // ── 레이블 → Cluster 구조체 변환 ────────────────────────────────
    if (cluster_id == 0) return;

    clusters.resize(cluster_id);
    for (int c = 0; c < cluster_id; ++c) {
        clusters[c].centroid_x_mm = 0.0;
        clusters[c].centroid_y_mm = 0.0;
    }

    for (size_t i = 0; i < n; ++i) {
        int lbl = labels[i];
        if (lbl < 0) continue; // NOISE는 어느 클러스터에도 속하지 않음

        clusters[lbl].points.push_back(points[i]);
    }

    // 중심점(centroid) 계산
    for (auto& cl : clusters) {
        if (cl.points.empty()) continue;

        double sum_x = 0.0, sum_y = 0.0;
        for (const auto& pt : cl.points) {
            sum_x += pt.x_mm;
            sum_y += pt.y_mm;
        }
        cl.centroid_x_mm = sum_x / cl.points.size();
        cl.centroid_y_mm = sum_y / cl.points.size();
    }

    // 빈 클러스터 제거 (border point만으로 구성된 경우는 드물지만 방어)
    clusters.erase(
        std::remove_if(clusters.begin(), clusters.end(),
                       [](const Cluster& c) { return c.points.empty(); }),
        clusters.end()
    );

    // 클러스터 폭(width) 계산: 클러스터 내 가장 먼 두 점 사이 거리
    for (auto& cl : clusters) {
        double max_w = 0.0;
        for (size_t i = 0; i < cl.points.size(); ++i) {
            for (size_t j = i + 1; j < cl.points.size(); ++j) {
                double dx = cl.points[i].x_mm - cl.points[j].x_mm;
                double dy = cl.points[i].y_mm - cl.points[j].y_mm;
                double d2 = dx * dx + dy * dy;
                if (d2 > max_w) max_w = d2;
            }
        }
        cl.width_mm = std::sqrt(max_w);
    }

    // 폭 기반 필터링
    clusters.erase(
        std::remove_if(clusters.begin(), clusters.end(),
                       [this](const Cluster& c) {
                           return c.width_mm < filter_.min_cluster_width_mm ||
                                  c.width_mm > filter_.max_cluster_width_mm;
                       }),
        clusters.end()
    );

    // 포인트 수 기준 내림차순 정렬 (가장 큰 클러스터 먼저)
    std::sort(clusters.begin(), clusters.end(),
              [](const Cluster& a, const Cluster& b) {
                  return a.points.size() > b.points.size();
              });
}

// ── 근접 클러스터 병합 ──────────────────────────────────────────────
//
//   보행자의 두 다리가 DBSCAN에서 별도 클러스터로 분리되는 문제 해결:
//     - 성인 보행 시 다리 간격: 약 150~300mm
//     - merge_radius_mm (예: 300mm) 이내 centroid를 가진 클러스터를 병합
//
//   알고리즘:
//     1. Union-Find로 merge_radius 이내 클러스터 쌍을 그룹화
//     2. 같은 그룹의 클러스터를 하나로 합침 (포인트 통합 + centroid/width 재계산)
//
void ScanProcessor::MergeClusters(std::vector<Cluster>& clusters) {
    const size_t n = clusters.size();
    if (n < 2) return;

    const double merge_sq = filter_.merge_radius_mm * filter_.merge_radius_mm;

    // Union-Find
    std::vector<size_t> parent(n);
    std::iota(parent.begin(), parent.end(), 0);

    std::function<size_t(size_t)> find = [&](size_t x) -> size_t {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    };

    auto unite = [&](size_t a, size_t b) {
        a = find(a);
        b = find(b);
        if (a != b) parent[b] = a;
    };

    // centroid 간 거리가 merge_radius 이내이고 폭 비율 조건을 만족하는 쌍만 병합
    // (폭 비율 체크: 사람 다리끼리는 비슷한 폭 → 병합 OK,
    //  사람+봉투는 폭 차이 큼 → 병합 차단)
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            double dx = clusters[i].centroid_x_mm - clusters[j].centroid_x_mm;
            double dy = clusters[i].centroid_y_mm - clusters[j].centroid_y_mm;
            if (dx * dx + dy * dy <= merge_sq) {
                double w1 = clusters[i].width_mm;
                double w2 = clusters[j].width_mm;
                if (w1 > 0.0 && w2 > 0.0) {
                    double ratio = (w1 > w2) ? w1 / w2 : w2 / w1;
                    if (ratio > filter_.merge_max_width_ratio) continue;
                }
                unite(i, j);
            }
        }
    }

    // 그룹 ID → 인덱스 매핑으로 포인트를 모아 새 클러스터 생성
    std::vector<Cluster> merged;
    std::vector<int> group_map(n, -1);

    for (size_t i = 0; i < n; ++i) {
        size_t root = find(i);

        if (group_map[root] < 0) {
            group_map[root] = static_cast<int>(merged.size());
            merged.push_back(Cluster{});
        }

        int mi = group_map[root];
        merged[mi].points.insert(merged[mi].points.end(),
                                 clusters[i].points.begin(),
                                 clusters[i].points.end());
    }

    // centroid + width 재계산
    for (auto& cl : merged) {
        if (cl.points.empty()) continue;

        double sum_x = 0.0, sum_y = 0.0;
        for (const auto& pt : cl.points) {
            sum_x += pt.x_mm;
            sum_y += pt.y_mm;
        }
        cl.centroid_x_mm = sum_x / cl.points.size();
        cl.centroid_y_mm = sum_y / cl.points.size();

        double max_w = 0.0;
        for (size_t i = 0; i < cl.points.size(); ++i) {
            for (size_t j = i + 1; j < cl.points.size(); ++j) {
                double dx = cl.points[i].x_mm - cl.points[j].x_mm;
                double dy = cl.points[i].y_mm - cl.points[j].y_mm;
                double d2 = dx * dx + dy * dy;
                if (d2 > max_w) max_w = d2;
            }
        }
        cl.width_mm = std::sqrt(max_w);
    }

    clusters = std::move(merged);
}

} // namespace ld19
