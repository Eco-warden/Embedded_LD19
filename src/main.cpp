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
 * @file  main.cpp
 * @brief LD19 LiDAR — 전체 파이프라인 통합
 * @date  2026
 *
 * 파이프라인:
 *   LiDAR scan → filter (거리/FOV/강도) → cartesian → DBSCAN
 *     → background filter → cluster tracker
 *       ├→ departure event → HTTP POST (FastAPI)
 *       ├→ dumping  event → HTTP POST (FastAPI)
 *       └→ point cloud + clusters → UDP (Unity)
 *
 * 빌드:
 *   sudo apt install -y libcurl4-openssl-dev
 *   mkdir build && cd build && cmake .. && make -j$(nproc)
 *
 * 실행:
 *   sudo ./ld19_lidar_app [시리얼포트] [API_URL] [Unity_IP:PORT]
 */

#include "ld19_lidar.h"
#include "scan_processor.h"
#include "background_filter.h"
#include "cluster_tracker.h"
#include "event_notifier.h"
#include "udp_sender.h"
#include "json_packet.h"

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <string>
#include <unistd.h>

// ── Ctrl+C 처리 ─────────────────────────────────────────────────────
static volatile sig_atomic_t g_running = 1;

static void SignalHandler(int /*sig*/) {
    g_running = 0;
}

// ── "IP:PORT" 문자열 파싱 ───────────────────────────────────────────
static bool ParseIpPort(const char* str, std::string& ip, uint16_t& port) {
    const char* colon = std::strrchr(str, ':');
    if (!colon) return false;

    ip.assign(str, colon);
    port = static_cast<uint16_t>(std::atoi(colon + 1));
    return port > 0;
}

// ── 클러스터 요약 출력 ──────────────────────────────────────────────
static void PrintClusterSummary(const std::vector<ld19::Cluster>& clusters,
                                size_t valid_count) {
    std::printf("  Filtered: %zu | Clusters: %zu\n",
                valid_count, clusters.size());

    for (size_t c = 0; c < clusters.size(); ++c) {
        const auto& cl = clusters[c];
        double dist = std::sqrt(cl.centroid_x_mm * cl.centroid_x_mm +
                                cl.centroid_y_mm * cl.centroid_y_mm);
        std::printf("    C%zu: (%+7.0f, %+7.0f) mm  d=%5.0f  w=%5.0f  pts=%zu\n",
                    c, cl.centroid_x_mm, cl.centroid_y_mm, dist,
                    cl.width_mm, cl.points.size());
    }
}

// ── 트랙 상태 출력 ──────────────────────────────────────────────────
static void PrintTracks(const std::vector<ld19::Track>& tracks) {
    if (tracks.empty()) return;

    std::printf("  Tracks (%zu):\n", tracks.size());
    std::printf("    %4s  %10s  %8s  %8s  %6s  %4s  %8s  %5s\n",
                "ID", "State", "X(mm)", "Y(mm)", "StopN", "Age", "CumDist", "Dump?");

    for (const auto& tr : tracks) {
        std::printf("    %4u  %10s  %+8.0f  %+8.0f  %6u  %4u  %8.0f  %5s\n",
                    tr.id,
                    ld19::TrackStateToString(tr.state),
                    tr.x_mm, tr.y_mm,
                    tr.stationary_count, tr.age,
                    tr.cumulative_dist_mm,
                    tr.is_dumped_item ? "YES" : "no");
    }
}

// ── 이벤트 출력 ─────────────────────────────────────────────────────
static void PrintDepartureEvents(const std::vector<ld19::DepartureEvent>& events) {
    for (const auto& evt : events) {
        std::printf("  ** DEPARTURE: track=%u pos=(%+.0f,%+.0f) stop=%u frames **\n",
                    evt.track_id, evt.x_mm, evt.y_mm, evt.stationary_frames);
    }
}

static void PrintDumpingEvents(const std::vector<ld19::DumpingEvent>& events) {
    for (const auto& evt : events) {
        std::printf("  ** DUMPING CONFIRMED: person=%u → object=%u "
                    "pos=(%+.0f,%+.0f) **\n",
                    evt.person_track_id, evt.object_track_id,
                    evt.object_x_mm, evt.object_y_mm);
    }
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT,  SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    // ── 인자 파싱 ────────────────────────────────────────────────────
    // argv[1]: 시리얼 포트       (기본 /dev/ttyUSB0)
    // argv[2]: FastAPI URL       (기본 http://localhost:8000/api/v1/events)
    // argv[3]: Unity IP:PORT     (기본 127.0.0.1:9090)
    const char* serial_port = (argc > 1) ? argv[1] : "/dev/ttyAMA0";
    const char* api_url     = (argc > 2) ? argv[2]
                              : "https://lorinda-nonexponible-zita.ngrok-free.dev/api/dumping-event";
    const char* unity_addr  = (argc > 3) ? argv[3] : "127.0.0.1:9090";

    // ── LiDAR ────────────────────────────────────────────────────────
    ld19::Config cfg;
    cfg.serial_port        = serial_port;
    cfg.baudrate           = 230400;
    cfg.connect_timeout_ms = 3500;
    cfg.scan_timeout_ms    = 1500;
    cfg.enable_filter      = true;

    // ── 프로세서 (FOV/강도/폭 필터 포함) ─────────────────────────────
    ld19::FilterParams fp;
    fp.min_distance_mm      = 200;
    fp.max_distance_mm      = 5000;
    fp.min_intensity        = 15;
    fp.fov_min_deg          = 90.0f;
    fp.fov_max_deg          = 270.0f;
    fp.min_cluster_width_mm = 30.0;
    fp.max_cluster_width_mm = 800.0;
    fp.merge_radius_mm      = 250.0;   // 450->250mm: 과도한 병합 방지 (쓰레기가 사람으로 흡수되는 것 차단)

    ld19::DBSCANParams dp;
    dp.epsilon_mm = 150.0;  // 200->150: 점구름 연결 범위 정상화
    dp.min_points = 3;  // 5→3: 작은 물체(봉투 등) 감지율 향상

    ld19::ScanProcessor processor(fp, dp);

    // ── 배경 필터 ────────────────────────────────────────────────────
    ld19::BackgroundFilterParams bgp;
    bgp.learning_frames = 50;
    bgp.match_radius_mm = 150.0;
    bgp.add_after_frames = 600; // 100->600: 정지 물체가 배경으로 흡수되는 시간 지연 (약 60초)

    ld19::BackgroundFilter bg_filter(bgp);

    // ── 추적기 (투기 감지 포함) ──────────────────────────────────────
    ld19::TrackerParams tp;
    tp.stationary_threshold_mm     = 30.0;
    tp.departure_frame_count       = 3;
    tp.association_max_dist_mm     = 400.0;
    tp.lost_age_limit              = 10;
    tp.enable_dumping_detection    = true;
    tp.min_walk_dist_mm            = 100.0; // 150->100: 더욱 짧은 이동도 허용
    tp.min_age_for_dump            = 5;     
    tp.dump_stationary_frame_count = 30;
    tp.separation_max_dist_mm      = 600.0; // 400->600: 쓰레기 분리 시 과거 궤적 참조 범위를 대폭 확대 (가장 중요)
    tp.separation_min_dist_from_current_mm = 150.0;  
    tp.min_dump_candidate_width_mm = 30.0;           
    tp.separation_confirm_frames   = 3;              
    tp.leg_proximity_radius_mm     = 200.0;          // 250->200: 분리된 쓰레기가 다시 다리로 오인되는 상황 차단 추가 완화
    tp.position_history_size       = 30;             // 궤적 이력 보존 프레임 수
    tp.recovery_max_dist_mm        = 600.0;          // 잠시 lost된 트랙 복구 최대 거리
    tp.recovery_max_lost_frames    = 3;              // 복구 허용 최대 lost 프레임

    ld19::ClusterTracker tracker(tp);

    // ── HTTP 전송기 ──────────────────────────────────────────────────
    ld19::NotifierConfig nc;
    nc.endpoint_url       = api_url;
    nc.queue_file_path    = "/tmp/ld19_event_queue.jsonl";
    nc.timeout_ms         = 3000;
    nc.max_retries        = 3;
    nc.retry_interval_sec = 10;

    ld19::EventNotifier notifier(nc);
    notifier.StartRetryThread();

    // ── UDP 송신기 ───────────────────────────────────────────────────
    ld19::UdpSenderConfig uc;
    std::string unity_ip;
    uint16_t    unity_port = 9090;
    if (ParseIpPort(unity_addr, unity_ip, unity_port)) {
        uc.dest_ip   = unity_ip;
        uc.dest_port = unity_port;
    } else {
        uc.dest_ip   = unity_addr;
        uc.dest_port = 9090;
    }

    ld19::UdpSender udp(uc);
    ld19::JsonPacketSender json_sender(udp);

    // ── 설정 출력 ────────────────────────────────────────────────────
    std::printf("=== LD19 LiDAR — Full Pipeline (with Dumping Detection) ===\n");
    std::printf("Serial     : %s @ %u\n", cfg.serial_port.c_str(), cfg.baudrate);
    std::printf("Filter     : %u ~ %u mm, intensity >= %u, FOV %.0f~%.0f°\n",
                fp.min_distance_mm, fp.max_distance_mm, fp.min_intensity,
                fp.fov_min_deg, fp.fov_max_deg);
    std::printf("Cluster    : width %.0f ~ %.0f mm, merge=%.0fmm\n",
                fp.min_cluster_width_mm, fp.max_cluster_width_mm,
                fp.merge_radius_mm);
    std::printf("DBSCAN     : eps=%.0fmm, minPts=%zu\n", dp.epsilon_mm, dp.min_points);
    std::printf("Background : %u learning frames, radius=%.0fmm\n",
                bgp.learning_frames, bgp.match_radius_mm);
    std::printf("Tracker    : stop=%.0fmm, depart=%u frames\n",
                tp.stationary_threshold_mm, tp.departure_frame_count);
    std::printf("Dumping    : walk>=%.0fmm, age>=%u, confirm=%u frames\n",
                tp.min_walk_dist_mm, tp.min_age_for_dump,
                tp.dump_stationary_frame_count);
    std::printf("Anti-leg   : minDist=%.0fmm, minWidth=%.0fmm, "
                "suspectFrames=%u, legRadius=%.0fmm\n",
                tp.separation_min_dist_from_current_mm,
                tp.min_dump_candidate_width_mm,
                tp.separation_confirm_frames,
                tp.leg_proximity_radius_mm);
    std::printf("HTTP API   : %s\n", nc.endpoint_url.c_str());
    std::printf("UDP Unity  : %s:%u\n", uc.dest_ip.c_str(), uc.dest_port);
    std::printf("\n");

    // ── UDP 소켓 열기 ────────────────────────────────────────────────
    if (!udp.Open()) {
        std::fprintf(stderr, "[ERROR] UDP socket open failed\n");
        notifier.StopRetryThread();
        return 1;
    }

    // ── 센서 시작 ────────────────────────────────────────────────────
    ld19::LD19Lidar lidar;

    ld19::Error err = lidar.Start(cfg);
    if (err != ld19::Error::None) {
        std::fprintf(stderr, "[ERROR] LiDAR start failed: %s\n",
                     ld19::ErrorToString(err));
        udp.Close();
        notifier.StopRetryThread();
        return 1;
    }

    std::printf("[INFO] LiDAR started. Press Ctrl+C to stop.\n\n");

    // ── 메인 루프 ────────────────────────────────────────────────────
    uint32_t frame_count = 0;

    while (g_running && lidar.IsRunning()) {
        ld19::ScanFrame frame;
        err = lidar.GetScanFrame(frame);

        if (err == ld19::Error::ScanTimeout) {
            std::fprintf(stderr, "[WARN] Scan timeout — retrying...\n");
            continue;
        }
        if (err != ld19::Error::None) {
            std::fprintf(stderr, "[ERROR] %s\n", ld19::ErrorToString(err));
            break;
        }
        if (frame.empty()) {
            usleep(50000);
            continue;
        }

        frame_count++;

        // 1) 필터 → 직교좌표 → DBSCAN
        std::vector<ld19::Cluster> clusters;
        size_t valid_count = processor.Process(frame, clusters);

        // 2) 투기물 위치를 배경 필터에 전달 (배경으로 흡수 방지)
        {
            std::vector<std::pair<double,double>> dump_positions;
            for (const auto& tr : tracker.GetTracks()) {
                if (tr.is_dumped_item || tr.is_dump_suspect) {
                    dump_positions.push_back({tr.x_mm, tr.y_mm});
                }
            }
            bg_filter.SetDumpedPositions(dump_positions);
        }

        // 3) 배경 필터 (학습 중이면 이 프레임 건너뜀)
        if (!bg_filter.Apply(clusters)) {
            if (frame_count % 10 == 0) {
                std::printf("[INFO] 배경 학습 중... (%u/%u)\n",
                            bg_filter.GetFrameCount(), bgp.learning_frames);
            }
            usleep(100000);
            continue;
        }

        // 필터링된 점구름 (직교좌표) 추출 — UDP 전송용
        std::vector<ld19::CartesianPoint> cart_points;
        for (const auto& cl : clusters) {
            cart_points.insert(cart_points.end(),
                               cl.points.begin(), cl.points.end());
        }

        // 3) 클러스터 추적 + 이탈/투기 감지
        std::vector<ld19::DepartureEvent> dep_events;
        std::vector<ld19::DumpingEvent>   dump_events;
        tracker.Update(clusters, dep_events, dump_events);

        // 4) 이탈 이벤트 → HTTP POST (테스트를 위해 비활성화)
        for (const auto& evt : dep_events) {
            // notifier.Send(evt);
        }

        // 5) 투기 이벤트 → HTTP POST (테스트를 위해 비활성화)
        for (const auto& evt : dump_events) {
            // notifier.SendDumping(evt);
        }

        // 6) 점구름 + 클러스터 → UDP 바이너리 (Unity 점구름 렌더링)
        udp.SendPoints(cart_points, frame_count);
        udp.SendClusters(clusters, tracker.GetTracks(), frame_count);

        // 7) 클러스터 + 이벤트 → UDP JSON (Unity 메타데이터)
        json_sender.Send(clusters, tracker.GetTracks(), dep_events, frame_count);

        // 8) 콘솔 출력 (투기 확정 이벤트만 노출되도록 상시 로그 주석 처리)
        // double freq_hz = 0.0;
        // lidar.GetScanFrequency(freq_hz);
        // std::printf("── Frame #%u  (raw=%zu, %.1f Hz) "
        //             "──────────────────────────────────\n",
        //             frame_count, frame.size(), freq_hz);
        // PrintClusterSummary(clusters, valid_count);
        // PrintTracks(tracker.GetTracks());

        if (!dep_events.empty()) {
            PrintDepartureEvents(dep_events);
        }
        if (!dump_events.empty()) {
            PrintDumpingEvents(dump_events);
        }

        // 통계 (20 프레임마다) 출력 주석 처리
        /*
        if (frame_count % 20 == 0) {
            std::printf("  [BG]   objects=%zu\n", bg_filter.GetBackgroundCount());
            // HTTP 전송 숨김
            std::printf("  [UDP]  pkts=%zu bytes=%zu errors=%zu\n",
                        udp.GetTotalPackets(), udp.GetTotalBytes(),
                        udp.GetSendErrors());
            std::printf("  [JSON] sent=%zu drop=%zu trunc=%zu\n",
                        json_sender.GetSentCount(),
                        json_sender.GetDropCount(),
                        json_sender.GetTruncCount());
        }
        std::printf("\n");
        */
        usleep(100000); // 100 ms (~10 Hz)
    }

    // ── 정리 ─────────────────────────────────────────────────────────
    std::printf("\n[INFO] Stopping...\n");
    notifier.StopRetryThread();
    lidar.Stop();
    udp.Close();

    size_t final_flush = notifier.FlushQueue();
    if (final_flush > 0) {
        std::printf("[INFO] Final flush: %zu events resent\n", final_flush);
    }

    std::printf("[INFO] Done.\n");
    return 0;
}
