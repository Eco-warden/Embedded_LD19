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
 * @file  cluster_tracker.cpp
 * @brief 프레임 간 클러스터 추적 + 객체 이탈 감지 + 투기 감지 구현
 * @date  2026
 *
 * 알고리즘 요약:
 *   1. 그리디 최근접 매칭으로 현재 클러스터를 기존 트랙에 연결
 *   2. 매칭된 트랙의 centroid 이동거리로 이동/정지 판정 + 누적 이동거리 갱신
 *   3. "이동 이력 있음 + 연속 정지 ≥ departure_frame_count" → 이탈 이벤트
 *   4. 미매칭 클러스터 중 기존 트랙 이전 위치 근처에 나타난 것 → 투기 의심 물체
 *   5. 투기 의심 물체가 dump_stationary_frame_count 이상 정지 → 투기 확정
 *   6. 매칭 실패 트랙은 lost_age_limit 초과 시 삭제
 */

#include "cluster_tracker.h"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdio>
#include <limits>

namespace ld19 {

// ── TrackState → 문자열 ─────────────────────────────────────────────
const char* TrackStateToString(TrackState s) {
    switch (s) {
        case TrackState::Moving:     return "MOVING";
        case TrackState::Stationary: return "STATIONARY";
        case TrackState::Departed:   return "DEPARTED";
        case TrackState::Lost:       return "LOST";
    }
    return "UNKNOWN";
}

// ── 현재 시각 (밀리초) ──────────────────────────────────────────────
uint64_t ClusterTracker::NowMs() {
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<milliseconds>(
            steady_clock::now().time_since_epoch()
        ).count()
    );
}

// ── 생성자 ──────────────────────────────────────────────────────────
ClusterTracker::ClusterTracker(const TrackerParams& params)
    : params_(params) {}

// ── 콜백 등록 ───────────────────────────────────────────────────────
void ClusterTracker::SetDepartureCallback(DepartureCallback cb) {
    dep_callback_ = std::move(cb);
}

void ClusterTracker::SetDumpingCallback(DumpingCallback cb) {
    dump_callback_ = std::move(cb);
}

// ── 그리디 최근접 매칭 ──────────────────────────────────────────────
void ClusterTracker::AssociateGreedy(
    const std::vector<Cluster>& clusters,
    std::vector<int>& cluster_to_track,
    std::vector<bool>& track_matched)
{
    const size_t nc = clusters.size();
    const size_t nt = tracks_.size();

    cluster_to_track.assign(nc, -1);
    track_matched.assign(nt, false);

    if (nc == 0 || nt == 0) return;

    const double max_dist_sq =
        params_.association_max_dist_mm * params_.association_max_dist_mm;

    struct Pair {
        double dist_sq;
        size_t ci;
        size_t ti;
    };
    std::vector<Pair> pairs;
    pairs.reserve(nc * nt);

    for (size_t c = 0; c < nc; ++c) {
        for (size_t t = 0; t < nt; ++t) {
            double dx = clusters[c].centroid_x_mm - tracks_[t].x_mm;
            double dy = clusters[c].centroid_y_mm - tracks_[t].y_mm;
            double d2 = dx * dx + dy * dy;
            if (d2 <= max_dist_sq) {
                pairs.push_back({d2, c, t});
            }
        }
    }

    std::sort(pairs.begin(), pairs.end(),
              [](const Pair& a, const Pair& b) {
                  return a.dist_sq < b.dist_sq;
              });

    for (const auto& p : pairs) {
        if (cluster_to_track[p.ci] != -1) continue;
        if (track_matched[p.ti])          continue;

        cluster_to_track[p.ci] = static_cast<int>(p.ti);
        track_matched[p.ti]    = true;
    }
}

// -- 클러스터 분열 감지 (Cluster Splitting) --
//
//   매칭 후, 각 트랙에 대해 매칭되지 않은 클러스터 중
//   트랙의 현재 위치 근처(트랙의 폭 + 여유)에 있는 것이 있으면
//   "1개 클러스터 → 2개 분열" 이벤트로 판정.
//   이 신호는 DetectSeparation에서 분리 판정 시 가중치로 사용.
//
void ClusterTracker::DetectClusterSplit(
    const std::vector<Cluster>& clusters,
    const std::vector<int>& cluster_to_track,
    const std::vector<bool>& track_matched)
{
    if (!params_.enable_dumping_detection) return;

    // 모든 트랙의 split 플래그 초기화
    for (auto& tr : tracks_) {
        tr.split_detected = false;
    }

    for (size_t t = 0; t < tracks_.size(); ++t) {
        if (!track_matched[t]) continue;
        auto& tr = tracks_[t];
        if (tr.is_dumped_item || tr.is_dump_suspect) continue;
        if (tr.cumulative_dist_mm < params_.min_walk_dist_mm) continue;

        // 이 트랙에 매칭된 클러스터를 찾아 기준점 설정
        double base_x = tr.x_mm;
        double base_y = tr.y_mm;

        // 분열 감지 반경: 트랙 폭 + 여유 (변경 가능)
        double split_radius = tr.width_mm + 200.0;  // 폭 + 20cm 여유
        double split_radius_sq = split_radius * split_radius;

        for (size_t c = 0; c < clusters.size(); ++c) {
            if (cluster_to_track[c] != -1) continue;  // 이미 매칭된 것 제외
            if (clusters[c].points.size() < params_.min_dump_candidate_points) continue;
            if (clusters[c].width_mm < params_.min_dump_candidate_width_mm) continue;

            double dx = clusters[c].centroid_x_mm - base_x;
            double dy = clusters[c].centroid_y_mm - base_y;
            double dist_sq = dx * dx + dy * dy;

            if (dist_sq < split_radius_sq) {
                tr.split_detected = true;
                tr.split_x_mm = clusters[c].centroid_x_mm;
                tr.split_y_mm = clusters[c].centroid_y_mm;
                std::printf("[INFO] 클러스터 분열 감지: "
                            "ID %u → 분열 위치 (%.0f, %.0f)mm\n",
                            tr.id, tr.split_x_mm, tr.split_y_mm);
                break;  // 트랙당 1개만 감지
            }
        }
    }
}

// -- 미매칭 클러스터에서 투기 의심 물체 분리 감지 --
//
//   개선된 분리 감지 로직:
//     (1) 트랙의 궤적 이력(position_history) 근처에 나타남 — 단일 prev 대신 전체 궤적
//     (2) 트랙의 현재 위치(current)에서 충분히 떨어져 있음 — 다리 필터
//     (3) 클러스터 폭이 최소 기준 이상 — 다리 하나가 아닌 물체
//     (4) 즉시 확정하지 않고 suspect 상태로 N프레임 독립 존재 확인
//
//   변경점:
//     - leg_proximity 사전 차단 제거 → 확정 단계(step 4)에서만 검증
//     - prev_x/prev_y 단일 비교 → position_history 전체 궤적 비교
//     - 분리 후보만 claim, 비후보는 step 6에서 일반 트랙 생성
//
void ClusterTracker::DetectSeparation(
    const std::vector<Cluster>& clusters,
    const std::vector<int>& cluster_to_track,
    std::vector<bool>& cluster_claimed)
{
    if (!params_.enable_dumping_detection) return;

    const double sep_prev_sq =
        params_.separation_max_dist_mm * params_.separation_max_dist_mm;
    const double sep_cur_sq =
        params_.separation_min_dist_from_current_mm *
        params_.separation_min_dist_from_current_mm;

    // 오인식 방지: 재등장한 기존 물체(최근 lost된 트랙 위치)를 새 투기물로 오인하지 않도록
    // lost 트랙의 위치와 겹치는 미매칭 클러스터는 분리 후보에서 제외
    const double recovery_sq =
        params_.recovery_max_dist_mm * params_.recovery_max_dist_mm;

    for (size_t c = 0; c < clusters.size(); ++c) {
        if (cluster_to_track[c] != -1) continue;
        if (cluster_claimed[c])        continue;

        // 조건 (3): 클러스터 폭 최소 기준
        if (clusters[c].width_mm < params_.min_dump_candidate_width_mm) continue;

        // 조건 (4): 점구름 개수 최소 기준 (노이즈 필터)
        if (clusters[c].points.size() < params_.min_dump_candidate_points) continue;

        // 오인식 방지: 최근 lost된 일반 트랙 위치와 겹치면 기존 물체 재등장이므로 건너뜀
        bool matches_lost_track = false;
        // (1) 현재 lost 상태인 트랙과 비교
        for (const auto& lt : tracks_) {
            if (lt.lost_count < 1) continue;
            if (lt.is_dumped_item || lt.is_dump_suspect) continue;
            double rdx = clusters[c].centroid_x_mm - lt.x_mm;
            double rdy = clusters[c].centroid_y_mm - lt.y_mm;
            if (rdx * rdx + rdy * rdy < recovery_sq) {
                matches_lost_track = true;
                break;
            }
        }
        // (2) 이미 삭제된 트랙의 최근 위치 버퍼와도 비교
        if (!matches_lost_track) {
            for (const auto& dp : deleted_positions_) {
                double rdx = clusters[c].centroid_x_mm - dp.x_mm;
                double rdy = clusters[c].centroid_y_mm - dp.y_mm;
                if (rdx * rdx + rdy * rdy < recovery_sq) {
                    matches_lost_track = true;
                    break;
                }
            }
        }
        if (matches_lost_track) continue;

        // 조건 (1)+(2): 궤적 이력 근처 + 현재 위치에서 떨어짐
        bool is_separation = false;
        int  person_id     = -1;

        for (const auto& tr : tracks_) {
            if (tr.is_dumped_item || tr.is_dump_suspect) continue;
            if (tr.cumulative_dist_mm < params_.min_walk_dist_mm) continue;
            if (tr.age < params_.min_age_for_dump) continue;

            // 현재 위치에서 충분히 떨어져 있어야 함
            double dx_cur = clusters[c].centroid_x_mm - tr.x_mm;
            double dy_cur = clusters[c].centroid_y_mm - tr.y_mm;
            double dist_cur_sq = dx_cur * dx_cur + dy_cur * dy_cur;
            if (dist_cur_sq < sep_cur_sq) continue;

            // prev 위치 체크
            double dx_prev = clusters[c].centroid_x_mm - tr.prev_x_mm;
            double dy_prev = clusters[c].centroid_y_mm - tr.prev_y_mm;
            if (dx_prev * dx_prev + dy_prev * dy_prev < sep_prev_sq) {
                is_separation = true;
                person_id = static_cast<int>(tr.id);
                break;
            }

            // 궤적 이력 전체 검색 — 사람이 지나간 경로 위에 물체가 나타났는지
            for (const auto& pos : tr.position_history) {
                double dhx = clusters[c].centroid_x_mm - pos.first;
                double dhy = clusters[c].centroid_y_mm - pos.second;
                if (dhx * dhx + dhy * dhy < sep_prev_sq) {
                    is_separation = true;
                    person_id = static_cast<int>(tr.id);
                    break;
                }
            }
            if (is_separation) break;
        }

        if (!is_separation) {
            // 폭 감소 보조 신호
            for (const auto& tr : tracks_) {
                if (!tr.width_drop_detected) continue;
                if (tr.is_dumped_item || tr.is_dump_suspect) continue;

                double dwx = clusters[c].centroid_x_mm - tr.width_drop_x_mm;
                double dwy = clusters[c].centroid_y_mm - tr.width_drop_y_mm;
                if (dwx * dwx + dwy * dwy < sep_prev_sq) {
                    is_separation = true;
                    person_id = static_cast<int>(tr.id);
                    std::printf("[INFO] 폭-감소 위치 기반 분리 감지 "
                                "(주체 ID: %d)\n", person_id);
                    break;
                }
            }
        }

        if (!is_separation) {
            // 클러스터 분열 보조 신호: 트랙의 split_detected 위치와 일치하면 분리 판정
            for (const auto& tr : tracks_) {
                if (!tr.split_detected) continue;
                if (tr.is_dumped_item || tr.is_dump_suspect) continue;

                double dsx = clusters[c].centroid_x_mm - tr.split_x_mm;
                double dsy = clusters[c].centroid_y_mm - tr.split_y_mm;
                // 분열 위치와 150mm 이내면 일치
                if (dsx * dsx + dsy * dsy < 150.0 * 150.0) {
                    is_separation = true;
                    person_id = static_cast<int>(tr.id);
                    std::printf("[INFO] 클러스터 분열 기반 분리 감지 "
                                "(주체 ID: %d)\n", person_id);
                    break;
                }
            }
        }

        if (!is_separation) continue;

        Track new_track{};
        new_track.id                    = AllocId();
        new_track.state                 = TrackState::Stationary;
        new_track.x_mm                  = clusters[c].centroid_x_mm;
        new_track.y_mm                  = clusters[c].centroid_y_mm;
        new_track.prev_x_mm            = new_track.x_mm;
        new_track.prev_y_mm            = new_track.y_mm;
        new_track.width_mm             = clusters[c].width_mm;
        new_track.stationary_count     = 0;
        new_track.lost_count           = 0;
        new_track.age                  = 1;
        new_track.was_moving           = false;
        new_track.departure_fired      = false;
        new_track.cumulative_dist_mm   = 0.0;
        new_track.is_dump_suspect      = true;
        new_track.is_dumped_item       = false;
        new_track.source_id            = person_id;
        new_track.source_x_mm          = 0.0;
        new_track.source_y_mm          = 0.0;
        new_track.source_cumulative_dist_mm = 0.0;
        new_track.dump_alert_fired     = false;
        new_track.suspect_confirm_count = 0;

        // 분리 시점의 투기 주체 위치 저장 (주체가 FOV를 벗어나도 이벤트 생성 가능)
        for (const auto& tr2 : tracks_) {
            if (static_cast<int>(tr2.id) == person_id) {
                new_track.source_x_mm = tr2.x_mm;
                new_track.source_y_mm = tr2.y_mm;
                new_track.source_cumulative_dist_mm = tr2.cumulative_dist_mm;
                break;
            }
        }

        cluster_claimed[c] = true;
        tracks_.push_back(new_track);

        std::printf("[INFO] 투기 후보 감지 (주체 ID: %d → 후보 ID: %u, "
                    "폭=%.0fmm). 독립 존재 확인 중...\n",
                    person_id, new_track.id, clusters[c].width_mm);
    }
}

// ── 투기물 정지 확인 → 투기 확정 ────────────────────────────────────
void ClusterTracker::CheckDumpingConfirmation(std::vector<DumpingEvent>& dump_events) {
    if (!params_.enable_dumping_detection) return;

    for (auto& tr : tracks_) {
        if (!tr.is_dumped_item)     continue;
        if (tr.dump_alert_fired)    continue;
        if (tr.stationary_count < params_.dump_stationary_frame_count) continue;

        // 투기 주체 이탈 확인: 주체가 투기물에서 충분히 멀어졌거나 FOV를 떠났는지 확인
        bool person_departed = false;
        bool person_found = false;
        double p_x = 0.0, p_y = 0.0, p_cum = 0.0;

        for (const auto& p : tracks_) {
            if (static_cast<int>(p.id) == tr.source_id) {
                p_x   = p.x_mm;
                p_y   = p.y_mm;
                p_cum = p.cumulative_dist_mm;
                person_found = true;

                // 주체와 투기물 간 거리 계산
                double dx = p.x_mm - tr.x_mm;
                double dy = p.y_mm - tr.y_mm;
                double dist = std::sqrt(dx * dx + dy * dy);
                if (dist >= params_.person_depart_dist_mm) {
                    person_departed = true;
                }
                break;
            }
        }

        if (!person_found) {
            // 주체가 FOV를 떠났음 (이탈 처리)
            person_departed = true;
            std::fprintf(stderr, "[WARN] 투기 주체(ID: %d) 소실 — 저장된 위치로 이벤트 생성\n",
                         tr.source_id);
            p_x   = tr.source_x_mm;
            p_y   = tr.source_y_mm;
            p_cum = tr.source_cumulative_dist_mm;
        }

        // 주체가 아직 근처에 있으면 확정을 보류
        if (!person_departed) continue;

        DumpingEvent evt;
        evt.person_track_id          = static_cast<uint32_t>(tr.source_id);
        evt.person_x_mm              = p_x;
        evt.person_y_mm              = p_y;
        evt.person_cumulative_dist_mm = p_cum;
        evt.object_track_id          = tr.id;
        evt.object_x_mm              = tr.x_mm;
        evt.object_y_mm              = tr.y_mm;
        evt.timestamp_ms             = NowMs();

        dump_events.push_back(evt);

        if (dump_callback_) {
            dump_callback_(evt);
        }

        tr.dump_alert_fired = true;

        std::printf("\n[ALERT] 쓰레기 투기 최종 확정!\n"
                    " -> 투기 주체 ID : %d\n"
                    " -> 투기물 ID    : %u\n"
                    " -> 투기물 위치  : (%.0f, %.0f) mm\n\n",
                    tr.source_id, tr.id, tr.x_mm, tr.y_mm);
    }
}

// ── Update: 핵심 추적 루프 ──────────────────────────────────────────
void ClusterTracker::Update(const std::vector<Cluster>& clusters,
                            std::vector<DepartureEvent>& dep_events,
                            std::vector<DumpingEvent>& dump_events) {
    dep_events.clear();
    dump_events.clear();
    frame_count_++;

    // 매칭 전 기존 트랙 수 기록 (이후 추가된 트랙은 track_matched 범위 밖)
    const size_t original_track_count = tracks_.size();

    // ── 1단계: 매칭 ─────────────────────────────────────────────────
    std::vector<int>  cluster_to_track;
    std::vector<bool> track_matched;
    AssociateGreedy(clusters, cluster_to_track, track_matched);

    // ── 1.5단계: 클러스터 분열 감지 (1 트랙 → 2 클러스터) ───────────
    DetectClusterSplit(clusters, cluster_to_track, track_matched);

    const double thresh_sq =
        params_.stationary_threshold_mm * params_.stationary_threshold_mm;

    // ── 2단계: 매칭된 트랙 갱신 ─────────────────────────────────────
    for (size_t c = 0; c < clusters.size(); ++c) {
        int ti = cluster_to_track[c];
        if (ti < 0) continue;

        Track& tr = tracks_[ti];
        tr.prev_x_mm = tr.x_mm;
        tr.prev_y_mm = tr.y_mm;
        tr.x_mm      = clusters[c].centroid_x_mm;
        tr.y_mm      = clusters[c].centroid_y_mm;

        // 폭 변화 추적 (투기 보조 신호)
        double old_width = tr.width_mm;
        tr.width_mm  = clusters[c].width_mm;

        // 폭 급감 감지: 이동 중인 사람의 폭만 검사 (정지 중인 사람의 폭 변화는 무시)
        if (!tr.is_dumped_item && !tr.is_dump_suspect &&
            tr.was_moving && tr.age > 3 &&
            old_width > 0.0 &&
            (old_width - tr.width_mm) > params_.width_drop_threshold_mm)
        {
            tr.width_drop_detected = true;
            tr.width_drop_x_mm     = tr.prev_x_mm;
            tr.width_drop_y_mm     = tr.prev_y_mm;
            std::printf("[INFO] 폭 감소 감지: ID %u, %.0fmm → %.0fmm "
                        "(차이=%.0fmm, 위치=(%.0f,%.0f))\n",
                        tr.id, old_width, tr.width_mm,
                        old_width - tr.width_mm,
                        tr.width_drop_x_mm, tr.width_drop_y_mm);
        } else {
            tr.prev_width_mm = tr.width_mm;
        }

        tr.lost_count = 0;
        tr.age++;

        // 속도 벡터 계산
        tr.vx_mm = tr.x_mm - tr.prev_x_mm;
        tr.vy_mm = tr.y_mm - tr.prev_y_mm;

        // 궤적 이력 기록 (투기 분리 감지에 사용)
        if (!tr.is_dumped_item && !tr.is_dump_suspect) {
            tr.position_history.push_back({tr.prev_x_mm, tr.prev_y_mm});
            if (tr.position_history.size() > params_.position_history_size) {
                tr.position_history.erase(tr.position_history.begin());
            }
        }

        double dx = tr.x_mm - tr.prev_x_mm;
        double dy = tr.y_mm - tr.prev_y_mm;
        double move_sq = dx * dx + dy * dy;
        double move_dist = std::sqrt(move_sq);

        tr.cumulative_dist_mm += move_dist;

        if (move_sq >= thresh_sq) {
            tr.state            = TrackState::Moving;
            tr.stationary_count = 0;
            tr.was_moving       = true;
            tr.departure_fired  = false;
        } else {
            tr.stationary_count++;

            if (tr.state == TrackState::Moving) {
                tr.state = TrackState::Stationary;
            }

            if (tr.was_moving &&
                !tr.departure_fired &&
                tr.stationary_count >= params_.departure_frame_count)
            {
                tr.state           = TrackState::Departed;
                tr.departure_fired = true;

                DepartureEvent evt;
                evt.track_id          = tr.id;
                evt.timestamp_ms      = NowMs();
                evt.x_mm              = tr.x_mm;
                evt.y_mm              = tr.y_mm;
                evt.stationary_frames = tr.stationary_count;

                dep_events.push_back(evt);

                if (dep_callback_) {
                    dep_callback_(evt);
                }
            }
        }
    }

    // ── 3단계: 미매칭 트랙 → lost 카운트 증가 ──────────────────────
    //   original_track_count 범위만 순회 (이후 추가된 트랙은 새 트랙이라 해당 없음)
    for (size_t t = 0; t < original_track_count; ++t) {
        if (!track_matched[t]) {
            tracks_[t].lost_count++;

            if (tracks_[t].is_dump_suspect &&
                tracks_[t].lost_count > 2)
            {
                tracks_[t].is_dump_suspect      = false;
                tracks_[t].source_id            = -1;
                tracks_[t].suspect_confirm_count = 0;
            }
        }
    }

    // ── 4단계: 투기 후보(suspect) → 확정(dumped_item) 승격 ──────────
    //   leg_proximity 체크에서 투기 주체(source_id)는 제외:
    //   봉투를 버린 사람이 아직 근처에 있는 것은 당연하므로,
    //   source person 때문에 suspect가 취소되면 투기 감지가 불가능해짐.
    //   다른 사람의 다리와 겹치는 경우만 다리로 간주하여 취소.
    if (params_.enable_dumping_detection) {
        const double leg_prox_sq =
            params_.leg_proximity_radius_mm * params_.leg_proximity_radius_mm;

        for (auto& tr : tracks_) {
            if (!tr.is_dump_suspect) continue;
            if (tr.is_dumped_item)   continue;

            bool near_other_person = false;
            for (const auto& other : tracks_) {
                if (other.id == tr.id) continue;
                if (static_cast<int>(other.id) == tr.source_id) continue;  // 투기 주체 제외
                if (other.is_dumped_item || other.is_dump_suspect) continue;

                double dx = tr.x_mm - other.x_mm;
                double dy = tr.y_mm - other.y_mm;
                if (dx * dx + dy * dy < leg_prox_sq) {
                    near_other_person = true;
                    break;
                }
            }

            if (near_other_person) {
                tr.is_dump_suspect      = false;
                tr.source_id            = -1;
                tr.suspect_confirm_count = 0;
                continue;
            }

            tr.suspect_confirm_count++;

            if (tr.suspect_confirm_count >= params_.separation_confirm_frames) {
                tr.is_dump_suspect = false;
                tr.is_dumped_item  = true;

                // 속도 벡터 분석: 분리 객체가 사람에게서 멀어지는 방향(receding)인지 확인
                bool is_receding = false;
                for (const auto& p : tracks_) {
                    if (static_cast<int>(p.id) == tr.source_id) {
                        // 사람→객체 방향 벡터
                        double dx = tr.x_mm - p.x_mm;
                        double dy = tr.y_mm - p.y_mm;
                        double dist = std::sqrt(dx * dx + dy * dy);
                        if (dist > 1.0) {
                            // 정규화된 방향 벡터
                            double nx = dx / dist;
                            double ny = dy / dist;
                            // 객체 속도의 이탈 성분 (dot product)
                            double recede_speed = tr.vx_mm * nx + tr.vy_mm * ny;
                            if (recede_speed > params_.receding_velocity_threshold) {
                                is_receding = true;
                            }
                        }
                        break;
                    }
                }

                std::printf("[INFO] 투기 의심 확정 (ID: %u, 주체: %d, 이탈운동: %s). "
                            "정지 상태 검증 시작...\n",
                            tr.id, tr.source_id,
                            is_receding ? "Yes" : "No");
            }
        }
    }

    // ── 5단계: lost_age_limit 초과 트랙 삭제 ────────────────────────
    //   투기 확정 물체(is_dumped_item)는 3배 더 오래 유지
    //   삭제되는 일반 트랙의 위치를 기록하여 오인식 방지
    tracks_.erase(
        std::remove_if(tracks_.begin(), tracks_.end(),
                       [this](const Track& tr) {
                           uint32_t limit = params_.lost_age_limit;
                           if (tr.is_dumped_item && !tr.dump_alert_fired) {
                               limit *= 3;
                           }
                           if (tr.lost_count > limit) {
                               if (tr.is_dumped_item && tr.dump_alert_fired) {
                                   std::printf("[INFO] 투기물 회수/소실 처리 | ID: %u\n", tr.id);
                               }
                               // 일반 트랙 삭제 시 위치를 기록 (오인식 방지)
                               if (!tr.is_dumped_item && !tr.is_dump_suspect) {
                                   deleted_positions_.push_back({
                                       tr.x_mm, tr.y_mm,
                                       params_.position_history_size
                                   });
                               }
                               return true;
                           }
                           return false;
                       }),
        tracks_.end()
    );

    // 삭제된 트랙 위치 버퍼 수명 관리
    for (auto& dp : deleted_positions_) {
        if (dp.frames_remaining > 0) dp.frames_remaining--;
    }
    deleted_positions_.erase(
        std::remove_if(deleted_positions_.begin(), deleted_positions_.end(),
                       [](const DeletedTrackPos& dp) {
                           return dp.frames_remaining == 0;
                       }),
        deleted_positions_.end()
    );

    // ── 6단계: 미매칭 클러스터 → 새 트랙 (투기 분리 감지 포함) ─────
    std::vector<bool> cluster_claimed(clusters.size(), false);
    for (size_t c = 0; c < clusters.size(); ++c) {
        if (cluster_to_track[c] != -1) cluster_claimed[c] = true;
    }

    DetectSeparation(clusters, cluster_to_track, cluster_claimed);

    // 복구 반경 (잠시 lost된 트랙의 cumulative_dist 등을 계승)
    const double recovery_sq =
        params_.recovery_max_dist_mm * params_.recovery_max_dist_mm;

    for (size_t c = 0; c < clusters.size(); ++c) {
        if (cluster_claimed[c]) continue;

        Track new_track{};
        new_track.id                 = AllocId();
        new_track.state              = TrackState::Moving;
        new_track.x_mm              = clusters[c].centroid_x_mm;
        new_track.y_mm              = clusters[c].centroid_y_mm;
        new_track.prev_x_mm         = new_track.x_mm;
        new_track.prev_y_mm         = new_track.y_mm;
        new_track.width_mm          = clusters[c].width_mm;
        new_track.stationary_count  = 0;
        new_track.lost_count        = 0;
        new_track.age               = 1;
        new_track.was_moving        = false;
        new_track.departure_fired   = false;
        new_track.cumulative_dist_mm = 0.0;
        new_track.is_dump_suspect   = false;
        new_track.is_dumped_item    = false;
        new_track.source_id         = -1;
        new_track.dump_alert_fired  = false;
        new_track.suspect_confirm_count = 0;

        // 트랙 복구: 최근 lost된 트랙 중 가까운 것을 찾아 상태 계승
        // (사람이 몸을 숙이면 LiDAR 프로파일이 급변하여 매칭 실패 → 새 트랙 생성 시
        //  cumulative_dist가 0으로 리셋되는 문제 해결)
        double best_dist_sq = recovery_sq;
        int best_recovery = -1;

        for (size_t t = 0; t < tracks_.size(); ++t) {
            const auto& lt = tracks_[t];
            if (lt.lost_count < 1) continue;
            if (lt.lost_count > params_.recovery_max_lost_frames) continue;
            if (lt.is_dumped_item || lt.is_dump_suspect) continue;

            double rdx = clusters[c].centroid_x_mm - lt.x_mm;
            double rdy = clusters[c].centroid_y_mm - lt.y_mm;
            double rd2 = rdx * rdx + rdy * rdy;
            if (rd2 < best_dist_sq) {
                best_dist_sq = rd2;
                best_recovery = static_cast<int>(t);
            }
        }

        if (best_recovery >= 0) {
            auto& recovered = tracks_[best_recovery];
            new_track.cumulative_dist_mm = recovered.cumulative_dist_mm;
            new_track.was_moving         = recovered.was_moving;
            new_track.age                = recovered.age;
            new_track.position_history   = recovered.position_history;
            recovered.lost_count = params_.lost_age_limit + 1;  // 원본은 제거 예정
            std::printf("[INFO] 트랙 복구: lost ID %u → new ID %u "
                        "(cumDist=%.0fmm 계승)\n",
                        recovered.id, new_track.id,
                        new_track.cumulative_dist_mm);
        }

        tracks_.push_back(new_track);
    }

    // ── 7단계: 투기물 정지 확인 → 투기 확정 ────────────────────────
    CheckDumpingConfirmation(dump_events);
}

} // namespace ld19
