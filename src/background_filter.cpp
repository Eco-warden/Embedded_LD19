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
 * @file  background_filter.cpp
 * @brief 배경 학습 + 적응형 갱신 기반 고정 물체 필터링 구현
 * @date  2026
 */

#include "background_filter.h"

#include <cmath>
#include <cstdio>
#include <algorithm>

namespace ld19 {

// ── 생성자 ──────────────────────────────────────────────────────────
BackgroundFilter::BackgroundFilter(const BackgroundFilterParams& params)
    : params_(params) {}

// ── Apply: 학습/운용 분기 ───────────────────────────────────────────
bool BackgroundFilter::Apply(std::vector<Cluster>& clusters) {
    if (IsLearning()) {
        LearnFrame(clusters);
        frame_count_++;

        if (frame_count_ == params_.learning_frames) {
            // 학습 완료 → 모든 엔트리를 확정 배경으로 마킹
            for (auto& bg : background_map_) {
                bg.confirmed = true;
            }
            std::printf("[INFO] 배경 학습 완료. 등록된 고정 물체 수: %zu\n",
                        background_map_.size());
        }
        return false;
    }

    FilterBackground(clusters);

    // 적응형 배경 갱신 (필터링 후 남은 클러스터 기반)
    if (params_.enable_adaptive) {
        AdaptiveUpdate(clusters);
    }

    frame_count_++;
    return true;
}

// ── 학습: 클러스터를 배경 맵에 누적 ─────────────────────────────────
void BackgroundFilter::LearnFrame(const std::vector<Cluster>& clusters) {
    const double radius_sq = params_.match_radius_mm * params_.match_radius_mm;

    for (const auto& cl : clusters) {
        bool matched = false;

        for (auto& bg : background_map_) {
            double dx = cl.centroid_x_mm - bg.x_mm;
            double dy = cl.centroid_y_mm - bg.y_mm;

            if (dx * dx + dy * dy <= radius_sq) {
                bg.x_mm = (bg.x_mm + cl.centroid_x_mm) / 2.0;
                bg.y_mm = (bg.y_mm + cl.centroid_y_mm) / 2.0;
                bg.seen_count++;
                matched = true;
                break;
            }
        }

        if (!matched) {
            BackgroundEntry entry;
            entry.x_mm       = cl.centroid_x_mm;
            entry.y_mm       = cl.centroid_y_mm;
            entry.width_mm   = cl.width_mm;
            entry.seen_count  = 1;
            entry.absent_count = 0;
            entry.confirmed  = false;
            background_map_.push_back(entry);
        }
    }
}

// ── 운용: 배경 맵에 매칭되는 클러스터 제거 ──────────────────────────
//   매칭된 배경 엔트리의 absent_count를 리셋한다 (적응형 갱신 연동)
void BackgroundFilter::FilterBackground(std::vector<Cluster>& clusters) {
    const double radius_sq = params_.match_radius_mm * params_.match_radius_mm;

    clusters.erase(
        std::remove_if(clusters.begin(), clusters.end(),
            [&](const Cluster& cl) {
                for (auto& bg : background_map_) {
                    if (!bg.confirmed) continue;  // 미확정 후보는 필터링에 사용하지 않음
                    double dx = cl.centroid_x_mm - bg.x_mm;
                    double dy = cl.centroid_y_mm - bg.y_mm;
                    if (dx * dx + dy * dy <= radius_sq) {
                        bg.absent_count = 0;  // 관측됨 → absent 리셋
                        return true;
                    }
                }
                return false;
            }),
        clusters.end()
    );
}

// ── 적응형 배경 갱신 ────────────────────────────────────────────────
//
//   운용 단계에서:
//     1. 기존 배경 엔트리가 현재 프레임에서 관측되면 absent 리셋
//     2. 관측되지 않으면 absent_count 증가
//     3. absent_count > remove_after_absent → 배경에서 제거 (물체가 치워짐)
//     4. 필터링 후 남은 클러스터(전경) 중 배경 근처에 계속 있는 것 → 새 배경 후보
//        seen_count > add_after_frames → 배경 등록 (물체가 새로 놓임)
//
void BackgroundFilter::AdaptiveUpdate(const std::vector<Cluster>& foreground_clusters) {
    const double radius_sq = params_.match_radius_mm * params_.match_radius_mm;

    // 기존 배경 관측 여부 체크 — 필터링에서 제거된 클러스터는 여기 안 오므로
    // 원본 스캔의 전체 클러스터가 필요하지만, 성능상 간접 추정:
    // "배경 위치에 뭔가 있는지"를 전경 클러스터 기준으로만 판단하면 부정확.
    // 대신: 매 프레임마다 absent_count를 1씩 증가시키고,
    //        FilterBackground에서 매칭된 경우 리셋하는 방식 사용.

    // 1) 모든 배경 엔트리의 absent_count 증가
    for (auto& bg : background_map_) {
        bg.absent_count++;
    }

    // 2) absent_count > 제한 → 제거
    background_map_.erase(
        std::remove_if(background_map_.begin(), background_map_.end(),
            [this](const BackgroundEntry& bg) {
                return bg.absent_count > params_.remove_after_absent;
            }),
        background_map_.end()
    );

    // 3) 전경 클러스터가 기존 배경 후보와 가까우면 seen_count 증가
    //    아니면 새 후보로 등록
    for (const auto& cl : foreground_clusters) {
        bool matched_candidate = false;

        for (auto& bg : background_map_) {
            if (bg.confirmed) continue;  // 이미 확정된 배경은 건너뜀

            double dx = cl.centroid_x_mm - bg.x_mm;
            double dy = cl.centroid_y_mm - bg.y_mm;
            if (dx * dx + dy * dy <= radius_sq) {
                bg.seen_count++;
                bg.x_mm = (bg.x_mm + cl.centroid_x_mm) / 2.0;
                bg.y_mm = (bg.y_mm + cl.centroid_y_mm) / 2.0;
                matched_candidate = true;

                // seen_count 임계값 초과 → 확정 배경 승격
                // 단, 투기 의심/확정 물체 위치는 배경 등록에서 제외
                if (bg.seen_count >= params_.add_after_frames) {
                    bool near_dump = false;
                    for (const auto& dp : dumped_positions_) {
                        double ddx = bg.x_mm - dp.first;
                        double ddy = bg.y_mm - dp.second;
                        if (ddx * ddx + ddy * ddy <= radius_sq) {
                            near_dump = true;
                            break;
                        }
                    }
                    if (near_dump) {
                        bg.seen_count = 0;
                        break;
                    }
                    bg.confirmed = true;
                    std::printf("[INFO] 적응형 배경 등록: (%.0f, %.0f) mm\n",
                                bg.x_mm, bg.y_mm);
                }
                break;
            }
        }

        if (!matched_candidate) {
            // 새 후보 등록 (아직 배경이 아님 — 필터링에 영향 없음)
            BackgroundEntry entry;
            entry.x_mm        = cl.centroid_x_mm;
            entry.y_mm        = cl.centroid_y_mm;
            entry.width_mm    = cl.width_mm;
            entry.seen_count   = 1;
            entry.absent_count = 0;
            entry.confirmed   = false;
            background_map_.push_back(entry);
        }
    }
}

} // namespace ld19
