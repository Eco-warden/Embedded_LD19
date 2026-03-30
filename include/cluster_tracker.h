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
 * @file  cluster_tracker.h
 * @brief 프레임 간 클러스터 추적 및 객체 이탈(departure) 감지
 * @date  2026
 *
 * 상태 머신 (트랙 단위):
 *
 *   ┌──────────┐   이동거리 ≥ 50mm   ┌──────────┐
 *   │  MOVING  │◄────────────────────│STATIONARY│
 *   └────┬─────┘                     └────┬─────┘
 *        │  이동거리 < 50mm               │ 정지 3프레임 연속
 *        ▼                                ▼
 *   ┌──────────┐                     ┌──────────┐
 *   │STATIONARY│                     │ DEPARTED │ ← 이벤트 1회 발화
 *   └──────────┘                     └──────────┘
 *
 *   LOST: 매칭 실패 시 age_limit 초과 후 트랙 삭제
 */

#pragma once

#include "scan_processor.h"

#include <cstdint>
#include <vector>
#include <utility>
#include <chrono>
#include <functional>

namespace ld19 {

// ── 트랙 상태 ────────────────────────────────────────────────────────
enum class TrackState {
    Moving,      // 이동 중 (centroid 이동 ≥ threshold)
    Stationary,  // 정지 중 (centroid 이동 < threshold)
    Departed,    // 이탈 판정 완료 (이벤트 발화 후 전이)
    Lost,        // 매칭 실패 — 삭제 대기
};

const char* TrackStateToString(TrackState s);

// ── 이탈 이벤트 ──────────────────────────────────────────────────────
struct DepartureEvent {
    uint32_t track_id;         // 추적 ID
    uint64_t timestamp_ms;     // 이탈 판정 시각 (epoch ms)
    double   x_mm;             // 이탈 위치 X (mm)
    double   y_mm;             // 이탈 위치 Y (mm)
    uint32_t stationary_frames;// 정지 유지 프레임 수
};

// ── 투기 이벤트 ──────────────────────────────────────────────────────
struct DumpingEvent {
    uint32_t person_track_id;  // 투기 주체 트랙 ID
    double   person_x_mm;      // 투기 주체 위치 X
    double   person_y_mm;      // 투기 주체 위치 Y
    double   person_cumulative_dist_mm; // 투기 주체 누적 이동거리
    uint32_t object_track_id;  // 투기물 트랙 ID
    double   object_x_mm;      // 투기물 위치 X
    double   object_y_mm;      // 투기물 위치 Y
    uint64_t timestamp_ms;     // 확정 시각 (epoch ms)
};

// ── 개별 트랙 ────────────────────────────────────────────────────────
struct Track {
    uint32_t   id;
    TrackState state;
    double     x_mm;              // 현재 centroid X
    double     y_mm;              // 현재 centroid Y
    double     prev_x_mm;         // 이전 프레임 centroid X
    double     prev_y_mm;         // 이전 프레임 centroid Y
    double     width_mm = 0.0;    // 클러스터 폭
    uint32_t   stationary_count;  // 연속 정지 프레임 수
    uint32_t   lost_count;        // 연속 매칭 실패 프레임 수
    uint32_t   age;               // 총 생존 프레임 수
    bool       was_moving;        // 정지 전에 이동 이력이 있었는지
    bool       departure_fired;   // 이탈 이벤트를 이미 발화했는지

    // ── 투기 감지 관련 필드 ──────────────────────────────────────────
    double     cumulative_dist_mm = 0.0;  // 누적 이동거리 (mm)
    bool       is_dump_suspect    = false; // 투기 의심 후보 (확정 전)
    bool       is_dumped_item     = false; // 투기 의심 물체 확정
    int        source_id          = -1;    // 투기 주체 트랙 ID (-1: 해당없음)
    double     source_x_mm        = 0.0;  // 분리 시점의 투기 주체 위치 X
    double     source_y_mm        = 0.0;  // 분리 시점의 투기 주체 위치 Y
    double     source_cumulative_dist_mm = 0.0; // 분리 시점의 투기 주체 누적 거리
    bool       dump_alert_fired   = false; // 투기 확정 알림 발화 여부
    uint32_t   suspect_confirm_count = 0;  // 분리 후 독립 존재 프레임 카운터

    // ── 궤적 이력 (투기 분리 감지 강화) ──────────────────────────────
    std::vector<std::pair<double,double>> position_history;  // 최근 N 프레임 위치
};

// ── 추적기 파라미터 ──────────────────────────────────────────────────
struct TrackerParams {
    double   stationary_threshold_mm = 50.0;   // 정지 판단 이동거리 (mm)
    uint32_t departure_frame_count   = 3;      // 이탈 판정 연속 정지 프레임 수
    double   association_max_dist_mm = 500.0;   // 클러스터-트랙 매칭 최대 거리 (mm)
    uint32_t lost_age_limit          = 5;       // 매칭 실패 허용 프레임 수

    // ── 투기 감지 파라미터 ───────────────────────────────────────────
    bool     enable_dumping_detection      = true;   // 투기 감지 활성화
    double   min_walk_dist_mm              = 500.0;  // 투기 주체 최소 누적 이동거리
    uint32_t min_age_for_dump              = 20;     // 투기 주체 최소 생존 프레임
    uint32_t dump_stationary_frame_count   = 30;     // 투기물 정지 확인 프레임 수
    double   separation_max_dist_mm        = 400.0;  // 분리 감지 최대 거리 (이전 위치 기준)

    // ── 다리 오인식 방지 파라미터 ────────────────────────────────────
    double   separation_min_dist_from_current_mm = 200.0;  // 투기물이 주체 현재 위치에서 최소한 이 거리 이상 떨어져야 함
    double   min_dump_candidate_width_mm   = 50.0;   // 투기 후보 최소 폭 (다리 하나 ~30~80mm 이므로 이보다 큰 물체만)
    uint32_t separation_confirm_frames     = 5;      // 분리 감지 후 이 프레임 동안 독립 존재해야 투기 의심 확정
    double   leg_proximity_radius_mm       = 350.0;  // 확정 단계에서 이 거리 내 사람 트랙 있으면 다리로 간주

    // ── 궤적 이력 + 트랙 복구 파라미터 ────────────────────────────────
    uint32_t position_history_size         = 30;     // 보존할 최근 위치 수 (궤적 기반 분리 감지)
    double   recovery_max_dist_mm          = 600.0;  // 잠시 lost된 트랙 복구 최대 거리 (mm)
    uint32_t recovery_max_lost_frames      = 3;      // 복구 허용 최대 lost 프레임 수
};

// ── 이탈 이벤트 콜백 타입 ────────────────────────────────────────────
using DepartureCallback = std::function<void(const DepartureEvent&)>;
using DumpingCallback   = std::function<void(const DumpingEvent&)>;

// ── 클러스터 추적기 ──────────────────────────────────────────────────
class ClusterTracker {
public:
    explicit ClusterTracker(const TrackerParams& params = TrackerParams{});

    /**
     * @brief 새 프레임의 클러스터를 기존 트랙에 매칭하고 상태를 갱신한다.
     * @param clusters       현재 프레임의 DBSCAN 클러스터 결과
     * @param[out] dep_events   이번 프레임에서 발생한 이탈 이벤트
     * @param[out] dump_events  이번 프레임에서 확정된 투기 이벤트
     */
    void Update(const std::vector<Cluster>& clusters,
                std::vector<DepartureEvent>& dep_events,
                std::vector<DumpingEvent>& dump_events);

    void SetDepartureCallback(DepartureCallback cb);
    void SetDumpingCallback(DumpingCallback cb);

    const std::vector<Track>& GetTracks() const { return tracks_; }
    uint32_t GetFrameCount() const { return frame_count_; }

    void SetParams(const TrackerParams& p) { params_ = p; }

private:
    TrackerParams      params_;
    std::vector<Track> tracks_;
    uint32_t           next_id_     = 1;  // 0 예약 (untracked)
    uint32_t           frame_count_ = 0;
    DepartureCallback  dep_callback_;
    DumpingCallback    dump_callback_;

    // 삭제된 일반 트랙의 최근 위치 버퍼 (오인식 방지용)
    // 트랙 삭제 후에도 해당 위치를 일정 기간 기억하여
    // 재등장한 기존 물체가 새 투기물로 오인되지 않도록 함
    struct DeletedTrackPos {
        double x_mm;
        double y_mm;
        uint32_t frames_remaining;  // 남은 유효 프레임 수
    };
    std::vector<DeletedTrackPos> deleted_positions_;

    static uint64_t NowMs();

    uint32_t AllocId() {
        uint32_t id = next_id_++;
        if (next_id_ == 0) next_id_ = 1;  // UINT32_MAX 오버플로 시 1로 리셋 (0 예약)
        return id;
    }

    void AssociateGreedy(
        const std::vector<Cluster>& clusters,
        std::vector<int>& cluster_to_track,
        std::vector<bool>& track_matched
    );

    // 미매칭 클러스터에서 투기 의심 물체 분리 감지
    void DetectSeparation(
        const std::vector<Cluster>& clusters,
        const std::vector<int>& cluster_to_track,
        std::vector<bool>& cluster_claimed
    );

    // 투기물 정지 상태 확인 → 투기 확정
    void CheckDumpingConfirmation(std::vector<DumpingEvent>& dump_events);
};

} // namespace ld19
