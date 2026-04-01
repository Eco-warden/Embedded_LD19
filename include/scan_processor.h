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
 * @file  scan_processor.h
 * @brief 2D LiDAR 스캔 데이터 처리 — 노이즈 필터, 좌표 변환, DBSCAN 클러스터링
 * @date  2026
 *
 * 파이프라인:
 *   ScanFrame (극좌표) → 노이즈 필터 → 직교좌표 변환 → DBSCAN → Cluster 벡터
 */

#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

#include "ld19_lidar.h"

namespace ld19 {

// ── 직교좌표 포인트 ──────────────────────────────────────────────────
struct CartesianPoint {
  double x_mm;          // 직교좌표 X (mm)
  double y_mm;          // 직교좌표 Y (mm)
  float angle_deg;      // 원본 극좌표 각도 (°)
  uint16_t distance_mm; // 원본 극좌표 거리 (mm)
  uint8_t intensity;    // 원본 반사 강도
};

// ── 클러스터 ─────────────────────────────────────────────────────────
struct Cluster {
  double centroid_x_mm;  // 클러스터 중심 X (mm)
  double centroid_y_mm;  // 클러스터 중심 Y (mm)
  double width_mm = 0.0; // 클러스터 내 최대 점간 거리 (mm)
  std::vector<CartesianPoint> points;
};

// ── 필터 파라미터 ────────────────────────────────────────────────────
struct FilterParams {
  uint16_t min_distance_mm = 30;     // 최소 거리 (이하 제거)
  uint16_t max_distance_mm = 12000;  // 최대 거리 (LD19 최대 12m)
  uint8_t min_intensity = 0;         // 최소 반사 강도 (이하 제거)
  float fov_min_deg = 180.0f;          // FOV 최소 각도 (°)
  float fov_max_deg = 360.0f;        // FOV 최대 각도 (°)
  double min_cluster_width_mm = 0.0; // 최소 클러스터 폭 (이하 제거)
  double max_cluster_width_mm = 1e9; // 최대 클러스터 폭 (초과 제거)

  // ── 보행자 다리 병합 ────────────────────────────────────────────
  double merge_radius_mm =
      0.0; // centroid 간 이 거리 이내 클러스터를 병합 (0=비활성)
  double merge_max_width_ratio =
      2.5; // 폭 비율이 이 값 이하인 쌍만 병합 (다리끼리는 OK, 사람+봉투는 NO)

  // ── 정적 배경 필터 ────────────────────────────────────────────
  double static_bg_margin_mm = 100.0; // 배경 심도보다 100mm 이상 튀어나온(가까운) 점만 남김 (벽면 밀착 객체 분리)
};

// ── DBSCAN 파라미터 ──────────────────────────────────────────────────
struct DBSCANParams {
  double epsilon_mm = 150.0; // 이웃 탐색 반경 (mm)
  size_t min_points = 5;     // 코어 포인트 최소 이웃 수
};

// ── 스캔 프로세서 ────────────────────────────────────────────────────
class ScanProcessor {
public:
  explicit ScanProcessor(const FilterParams &filter = FilterParams{},
                         const DBSCANParams &dbscan = DBSCANParams{});

  /**
   * @brief 전체 파이프라인 실행: 필터 → 변환 → 클러스터링
   * @param raw     LiDAR 원본 극좌표 프레임
   * @param[out] clusters  클러스터 결과
   * @return 필터 통과 후 유효 포인트 수
   */
  size_t Process(const ScanFrame &raw, std::vector<Cluster> &clusters);

  /**
   * @brief 노이즈 필터: 유효 거리 범위 밖의 포인트 제거
   */
  void FilterNoise(const ScanFrame &raw, ScanFrame &filtered);

  /**
   * @brief 정적 배경(방 형태)의 깊이 맵을 학습합니다.
   *        0.1도 단위의 3600개 슬롯에 EMA(지수 이동 평균) 방식으로 거리를 기록.
   */
  void LearnStaticBackground(const ScanFrame &raw);

  /**
   * @brief 정적 배경 필터: 학습된 벽면과 인접한 포인트 삭제
   */
  void FilterStaticBackground(const ScanFrame &raw, ScanFrame &filtered);

  /**
   * @brief 극좌표 → 직교좌표 변환
   *        x = distance * cos(angle),  y = distance * sin(angle)
   */
  void ToCartesian(const ScanFrame &polar, std::vector<CartesianPoint> &cart);

  /**
   * @brief DBSCAN 클러스터링 수행
   */
  void ClusterDBSCAN(const std::vector<CartesianPoint> &points,
                     std::vector<Cluster> &clusters);

  /**
   * @brief DBSCAN 후 centroid 간 거리가 merge_radius_mm 이내인 클러스터를 병합
   *        보행자의 두 다리가 별도 클러스터로 분리되는 문제를 해결
   */
  void MergeClusters(std::vector<Cluster> &clusters);

  void SetFilterParams(const FilterParams &p) { filter_ = p; }
  void SetDBSCANParams(const DBSCANParams &p) { dbscan_ = p; }

private:
  FilterParams filter_;
  DBSCANParams dbscan_;

  // 360도 공간 0.1도 단위의 정적 배경 깊이 (방의 형태)
  uint16_t static_bg_depth_[3600] = {0};

  static constexpr double kDegToRad = M_PI / 180.0;

  // DBSCAN 내부 레이블
  static constexpr int UNVISITED = -1;
  static constexpr int NOISE = -2;

  // ── 그리드 기반 공간 인덱스 (DBSCAN 가속) ────────────────────────
  struct GridKey {
    int32_t gx;
    int32_t gy;
    bool operator==(const GridKey &o) const { return gx == o.gx && gy == o.gy; }
  };
  struct GridKeyHash {
    size_t operator()(const GridKey &k) const {
      return std::hash<int64_t>()((static_cast<int64_t>(k.gx) << 32) |
                                  static_cast<uint32_t>(k.gy));
    }
  };
  using GridIndex =
      std::unordered_map<GridKey, std::vector<size_t>, GridKeyHash>;

  void BuildGrid(const std::vector<CartesianPoint> &points, double cell_size,
                 GridIndex &grid);

  void RangeQueryGrid(const std::vector<CartesianPoint> &points,
                      const GridIndex &grid, double cell_size, size_t idx,
                      double eps_sq, std::vector<size_t> &neighbors);
};

} // namespace ld19
