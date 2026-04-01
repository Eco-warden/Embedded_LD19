/**
 * @file  sim_main.cpp
 * @brief LD19 시뮬레이션 — 센서 없이 투기 감지 파이프라인 테스트
 *
 * 시나리오:
 *   프레임 001-050 : 배경 학습 (빈 장면)
 *   프레임 051+    : 사람 진입 → 남쪽 이동 (100mm/frame)
 *   프레임 066     : 사람이 지나간 자리에 물체 출현 → 분리 감지
 *   프레임 071     : 투기 의심 확정 → 정지 검증 시작
 *   프레임 ~096    : 투기 최종 확정 → HTTP POST 전송
 *
 * 빌드:
 *   cd build && cmake .. && make -j$(nproc)
 * 실행:
 *   ./ld19_sim [API_URL]
 */

#include "scan_processor.h"
#include "background_filter.h"
#include "cluster_tracker.h"
#include "event_notifier.h"

#include <csignal>
#include <cstdio>
#include <cmath>
#include <vector>
#include <unistd.h>

// ── Ctrl+C 처리 ─────────────────────────────────────────────────────
static volatile sig_atomic_t g_running = 1;
static void SignalHandler(int) { g_running = 0; }

// ── 가상 클러스터용 스캔 포인트 생성 ─────────────────────────────────
//    (cx, cy) 중심, width_mm 폭, npts개 포인트를 극좌표로 생성
static void AddClusterPoints(double cx, double cy, double width_mm,
                             int npts, ld19::ScanFrame& frame) {
    double dist   = std::sqrt(cx * cx + cy * cy);
    double angle  = std::atan2(cy, cx);            // radians
    double half   = std::atan2(width_mm / 2.0, dist);

    for (int i = 0; i < npts; i++) {
        double t = (npts <= 1) ? 0.0
                   : (static_cast<double>(i) / (npts - 1) * 2.0 - 1.0);
        double a = angle + t * half;
        double d = dist + ((i % 2 == 0) ? 12.0 : -12.0);

        float deg = static_cast<float>(a * 180.0 / M_PI);
        if (deg < 0) deg += 360.0f;

        ld19::ScanPoint sp;
        sp.angle_deg   = deg;
        sp.distance_mm = static_cast<uint16_t>(std::max(1.0, d));
        sp.intensity   = 100;
        frame.push_back(sp);
    }
}

// ── 콘솔 출력 헬퍼 (main.cpp와 동일) ────────────────────────────────
static void PrintClusters(const std::vector<ld19::Cluster>& clusters,
                          size_t valid) {
    std::printf("  Filtered: %zu | Clusters: %zu\n", valid, clusters.size());
    for (size_t c = 0; c < clusters.size(); ++c) {
        const auto& cl = clusters[c];
        double d = std::sqrt(cl.centroid_x_mm * cl.centroid_x_mm +
                             cl.centroid_y_mm * cl.centroid_y_mm);
        std::printf("    C%zu: (%+7.0f, %+7.0f) mm  d=%5.0f  w=%5.0f  pts=%zu\n",
                    c, cl.centroid_x_mm, cl.centroid_y_mm, d,
                    cl.width_mm, cl.points.size());
    }
}

static void PrintTracks(const std::vector<ld19::Track>& tracks) {
    if (tracks.empty()) return;
    std::printf("  Tracks (%zu):\n", tracks.size());
    std::printf("    %4s  %10s  %8s  %8s  %6s  %4s  %8s  %5s\n",
                "ID", "State", "X(mm)", "Y(mm)", "StopN", "Age", "CumDist", "Dump?");
    for (const auto& tr : tracks) {
        std::printf("    %4u  %10s  %+8.0f  %+8.0f  %6u  %4u  %8.0f  %5s\n",
                    tr.id, ld19::TrackStateToString(tr.state),
                    tr.x_mm, tr.y_mm,
                    tr.stationary_count, tr.age,
                    tr.cumulative_dist_mm,
                    tr.is_dumped_item ? "YES" : (tr.is_dump_suspect ? "sus" : "no"));
    }
}

static void PrintDepartures(const std::vector<ld19::DepartureEvent>& evts) {
    for (const auto& e : evts)
        std::printf("  ** DEPARTURE: track=%u pos=(%+.0f,%+.0f) stop=%u **\n",
                    e.track_id, e.x_mm, e.y_mm, e.stationary_frames);
}

static void PrintDumpings(const std::vector<ld19::DumpingEvent>& evts) {
    for (const auto& e : evts)
        std::printf("  ** DUMPING CONFIRMED: person=%u → object=%u "
                    "pos=(%+.0f,%+.0f) **\n",
                    e.person_track_id, e.object_track_id,
                    e.object_x_mm, e.object_y_mm);
}

// ═══════════════════════════════════════════════════════════════════════
int main(int argc, char* argv[]) {
    std::signal(SIGINT,  SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    const char* api_url = (argc > 1) ? argv[1]
        : "https://api.ecowarden.systems/api/dumping-event";

    // ── 파이프라인 파라미터 (main.cpp와 동일) ────────────────────────
    ld19::FilterParams fp;
    fp.min_distance_mm      = 200;
    fp.max_distance_mm      = 5000;
    fp.min_intensity        = 15;
    fp.fov_min_deg          = 90.0f;
    fp.fov_max_deg          = 270.0f;
    fp.min_cluster_width_mm = 10.0;
    fp.max_cluster_width_mm = 800.0;
    fp.merge_radius_mm      = 300.0;

    ld19::DBSCANParams dp;
    dp.epsilon_mm = 250.0;
    dp.min_points = 2;

    ld19::ScanProcessor processor(fp, dp);

    ld19::ScanProcessor processor(fp, dp);

    ld19::BackgroundFilterParams bgp;
    bgp.learning_frames = 50;
    bgp.match_radius_mm = 150.0;
    ld19::BackgroundFilter bg_filter(bgp);

    ld19::TrackerParams tp;
    tp.stationary_threshold_mm     = 30.0;
    tp.departure_frame_count       = 50;
    tp.association_max_dist_mm     = 400.0;
    tp.lost_age_limit              = 10;
    tp.enable_dumping_detection    = true;
    tp.min_walk_dist_mm            = 500.0;
    tp.min_age_for_dump            = 10;
    tp.dump_stationary_frame_count = 30;
    tp.separation_max_dist_mm      = 400.0;
    tp.separation_min_dist_from_current_mm = 200.0;
    tp.min_dump_candidate_width_mm = 10.0;
    tp.separation_confirm_frames   = 5;
    tp.leg_proximity_radius_mm     = 350.0;
    tp.position_history_size       = 30;
    tp.recovery_max_dist_mm        = 150.0;
    tp.recovery_max_lost_frames    = 3;
    ld19::ClusterTracker tracker(tp);

    ld19::NotifierConfig nc;
    nc.endpoint_url       = api_url;
    nc.queue_file_path    = "/tmp/ld19_sim_event_queue.jsonl";
    nc.timeout_ms         = 3000;
    nc.max_retries        = 3;
    nc.retry_interval_sec = 10;
    ld19::EventNotifier notifier(nc);
    notifier.StartRetryThread();

    // ── 시나리오 파라미터 ────────────────────────────────────────────
    const double person_x      = -2000.0;  // 사람 X 고정
    double       person_y      =  1200.0;  // 사람 Y 시작
    const double person_speed  =   100.0;  // mm/frame 남쪽 이동
    const double object_x      = -2000.0;  // 투기물 X
    const double object_y      =   300.0;  // 투기물 Y (사람이 프레임60에 지나간 자리)
    const int    object_frame  =       66; // 물체 출현 프레임
    const int    total_frames  =      105; // 총 프레임

    // ── 안내 출력 ────────────────────────────────────────────────────
    std::printf("\n");
    std::printf("╔══════════════════════════════════════════════════════╗\n");
    std::printf("║   LD19 시뮬레이션 — 투기 감지 시나리오               ║\n");
    std::printf("╠══════════════════════════════════════════════════════╣\n");
    std::printf("║  프레임 001-050 : 배경 학습 (빈 장면)               ║\n");
    std::printf("║  프레임 051+    : 사람 진입 → 남쪽 이동             ║\n");
    std::printf("║  프레임 066     : 투기물 출현 → 분리 감지           ║\n");
    std::printf("║  프레임 ~071    : 투기 의심 확정                    ║\n");
    std::printf("║  프레임 ~096    : 투기 최종 확정 → HTTP POST        ║\n");
    std::printf("╠══════════════════════════════════════════════════════╣\n");
    std::printf("║  사람 경로 : (%+.0f, %+.0f) → 남쪽 %.0fmm/frame    ║\n",
                person_x, person_y, person_speed);
    std::printf("║  투기물    : (%+.0f, %+.0f) 고정                    ║\n",
                object_x, object_y);
    std::printf("║  HTTP API  : %s  ║\n", api_url);
    std::printf("╚══════════════════════════════════════════════════════╝\n\n");

    // ── 메인 루프 ────────────────────────────────────────────────────
    bool dumping_confirmed = false;

    for (int f = 1; f <= total_frames && g_running; f++) {

        // 1) 가상 스캔 프레임 생성
        ld19::ScanFrame frame;

        if (f >= 51 && person_y > -4500.0) {
            AddClusterPoints(person_x, person_y, 300.0, 5, frame);
        }
        if (f >= object_frame) {
            AddClusterPoints(object_x, object_y, 100.0, 4, frame);
        }

        // 2) 필터 → 직교좌표 → DBSCAN
        std::vector<ld19::Cluster> clusters;
        size_t valid = processor.Process(frame, clusters);

        // 3) 투기물 위치를 배경 필터에 전달
        {
            std::vector<std::pair<double,double>> dump_positions;
            for (const auto& tr : tracker.GetTracks()) {
                if (tr.is_dumped_item || tr.is_dump_suspect)
                    dump_positions.push_back({tr.x_mm, tr.y_mm});
            }
            bg_filter.SetDumpedPositions(dump_positions);
        }

        // 4) 배경 필터
        if (!bg_filter.Apply(clusters)) {
            if (f % 10 == 0)
                std::printf("[배경 학습] %u / %u 프레임\n",
                            bg_filter.GetFrameCount(), bgp.learning_frames);
            usleep(10000);  // 10ms — 학습은 빠르게
            continue;
        }

        // 5) 클러스터 추적 + 이탈/투기 감지
        std::vector<ld19::DepartureEvent> dep_events;
        std::vector<ld19::DumpingEvent>   dump_events;
        tracker.Update(clusters, dep_events, dump_events);

        // 6) 이벤트 → HTTP POST
        for (const auto& e : dep_events)  notifier.Send(e);
        for (const auto& e : dump_events) notifier.SendDumping(e);

        // 7) 콘솔 출력
        std::printf("── Frame #%03d ──────────────────────────────────────\n", f);

        if (f >= 51 && person_y > -4500.0)
            std::printf("  [SIM] 사람 위치: (%+.0f, %+.0f) mm\n", person_x, person_y);
        if (f >= object_frame)
            std::printf("  [SIM] 투기물 위치: (%+.0f, %+.0f) mm\n", object_x, object_y);

        PrintClusters(clusters, valid);
        PrintTracks(tracker.GetTracks());

        if (!dep_events.empty())  PrintDepartures(dep_events);
        if (!dump_events.empty()) {
            PrintDumpings(dump_events);
            dumping_confirmed = true;
        }

        // HTTP 통계 (10프레임마다)
        if (f % 10 == 0) {
            std::printf("  [HTTP] sent=%zu fail=%zu queued=%zu\n",
                        notifier.GetSentCount(), notifier.GetFailCount(),
                        notifier.GetQueueSize());
        }

        std::printf("\n");

        // 사람 이동
        if (f >= 51) person_y -= person_speed;

        // 투기 확정 후 몇 프레임 더 보여주고 종료
        if (dumping_confirmed && f >= 100) {
            std::printf("── 시뮬레이션 완료: 투기 감지 성공 ──\n");
            break;
        }

        usleep(150000);  // 150ms
    }

    // ── 정리 ─────────────────────────────────────────────────────────
    notifier.StopRetryThread();

    std::printf("\n[결과] HTTP sent=%zu, fail=%zu, queued=%zu\n",
                notifier.GetSentCount(), notifier.GetFailCount(),
                notifier.GetQueueSize());
    std::printf("[INFO] 시뮬레이션 종료.\n");
    return 0;
}
