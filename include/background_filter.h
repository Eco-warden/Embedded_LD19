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
 * @file  background_filter.h
 * @brief 배경 학습 기반 고정 물체 필터링
 * @date  2026
 *
 * 동작:
 *   1. 학습 단계 (learning_frames 프레임 동안):
 *      매 프레임의 클러스터를 누적하여 배경 맵 구축
 *   2. 운용 단계:
 *      배경 맵에 등록된 위치와 가까운 클러스터를 제거
 */

#pragma once

#include "scan_processor.h"

#include <vector>
#include <utility>
#include <cstdint>

namespace ld19 {

// ── 배경 필터 파라미터 ──────────────────────────────────────────────
struct BackgroundFilterParams {
    uint32_t learning_frames = 50;      // 배경 학습 프레임 수
    double   match_radius_mm = 150.0;   // 배경 매칭 반경 (mm)

    // ── 적응형 배경 갱신 ────────────────────────────────────────────
    bool     enable_adaptive  = true;   // 운용 중 배경 자동 갱신 활성화
    uint32_t add_after_frames = 100;    // 이 프레임 동안 같은 위치에 있으면 배경 등록
    uint32_t remove_after_absent = 200; // 이 프레임 동안 안 보이면 배경에서 제거

    // ── 이중 배경 모델 (장기) ────────────────────────────────────────
    bool     enable_dual_background     = true;    // 이중 배경 모델 활성화
    uint32_t long_term_add_frames       = 500;     // 장기 배경 등록 프레임 수
    uint32_t long_term_remove_after     = 1000;    // 장기 배경 제거 absent 프레임
};

// ── 배경 맵 엔트리 ──────────────────────────────────────────────────
struct BackgroundEntry {
    double   x_mm;
    double   y_mm;
    double   width_mm;
    uint32_t seen_count  = 0;   // 연속 관측 프레임 수 (적응형 등록용)
    uint32_t absent_count = 0;  // 연속 미관측 프레임 수 (적응형 제거용)
    bool     confirmed   = false; // 학습 단계에서 등록된 확정 배경인지
};

// ── 배경 필터 ───────────────────────────────────────────────────────
class BackgroundFilter {
public:
    explicit BackgroundFilter(const BackgroundFilterParams& params = BackgroundFilterParams{});

    /**
     * @brief 클러스터를 배경 맵에 학습시키거나, 배경을 필터링한다.
     *
     * @param clusters  입력 클러스터 (운용 단계에서 배경 클러스터 제거됨)
     * @return true: 학습 완료 후 운용 단계, false: 아직 학습 중 (클러스터 무시)
     */
    bool Apply(std::vector<Cluster>& clusters);

    bool IsLearning() const { return frame_count_ < params_.learning_frames; }
    bool IsReady()    const { return !IsLearning(); }

    uint32_t GetFrameCount()      const { return frame_count_; }
    size_t   GetBackgroundCount() const { return background_map_.size(); }

    void SetParams(const BackgroundFilterParams& p) { params_ = p; }

    void SetDumpedPositions(const std::vector<std::pair<double,double>>& positions) {
        dumped_positions_ = positions;
    }

private:
    BackgroundFilterParams        params_;
    std::vector<BackgroundEntry>  background_map_;       // 단기 배경
    std::vector<BackgroundEntry>  long_term_map_;        // 장기 배경
    uint32_t                      frame_count_ = 0;
    std::vector<std::pair<double,double>> dumped_positions_;

    void LearnFrame(const std::vector<Cluster>& clusters);
    void FilterBackground(std::vector<Cluster>& clusters);
    void AdaptiveUpdate(const std::vector<Cluster>& foreground_clusters);

    bool IsInLongTermBg(double x, double y) const;
};

} // namespace ld19
