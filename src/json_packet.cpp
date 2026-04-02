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
 * @file  json_packet.cpp
 * @brief nlohmann/json 기반 클러스터+이벤트 JSON 직렬화 및 UDP 전송
 * @date  2026
 */

#include "json_packet.h"

#include <cstdio>
#include <ctime>
#include <chrono>
#include <algorithm>
#include <cmath>

using json = nlohmann::json;

namespace ld19 {

// ── 생성자 ──────────────────────────────────────────────────────────
JsonPacketSender::JsonPacketSender(UdpSender& udp) : udp_(udp) {}

// ── epoch ms → ISO 8601 ─────────────────────────────────────────────
std::string JsonPacketSender::MsToIso8601(uint64_t epoch_ms) {
    time_t sec = static_cast<time_t>(epoch_ms / 1000);
    uint32_t ms_part = static_cast<uint32_t>(epoch_ms % 1000);

    struct tm utc{};
    gmtime_r(&sec, &utc);

    char buf[64];
    std::snprintf(buf, sizeof(buf),
                  "%04d-%02d-%02dT%02d:%02d:%02d.%03uZ",
                  utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
                  utc.tm_hour, utc.tm_min, utc.tm_sec, ms_part);
    return buf;
}

// ── NowMs ───────────────────────────────────────────────────────────
uint64_t JsonPacketSender::NowMs() {
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<milliseconds>(
            system_clock::now().time_since_epoch()
        ).count()
    );
}

// ── 클러스터 타입 판별 ──────────────────────────────────────────────
//   클러스터 centroid에 가장 가까운 트랙을 찾아서
//   Departed 상태면 "abandoned", 아니면 "normal"
std::string JsonPacketSender::ResolveType(const Cluster& cluster,
                                          const std::vector<Track>& tracks) {
    if (tracks.empty()) return "normal";

    double best_sq = 1e18;
    TrackState best_state = TrackState::Moving;
    bool is_dumped = false;

    for (const auto& tr : tracks) {
        double dx = cluster.centroid_x_mm - tr.x_mm;
        double dy = cluster.centroid_y_mm - tr.y_mm;
        double d2 = dx * dx + dy * dy;
        if (d2 < best_sq) {
            best_sq    = d2;
            best_state = tr.state;
            is_dumped  = tr.is_dumped_item;
        }
    }

    if (is_dumped) return "dumped";
    return (best_state == TrackState::Departed) ? "abandoned" : "normal";
}

// ── Serialize: JSON 직렬화 ──────────────────────────────────────────
//
//   출력 형식:
//   {
//     "frame_id": 42,
//     "timestamp": "2026-03-27T12:34:56.789Z",
//     "clusters": [
//       {"id": 0, "x": 1200.0, "y": 350.0, "count": 12, "type": "normal"},
//       {"id": 1, "x": 800.0,  "y": 200.0, "count": 8,  "type": "abandoned"}
//     ],
//     "event": {
//       "type": "abandoned",
//       "x": 800.0,
//       "y": 200.0,
//       "cluster_id": 1,
//       "timestamp": "2026-03-27T12:34:56.789Z"
//     }
//   }
//
std::string JsonPacketSender::Serialize(
    const std::vector<Cluster>& clusters,
    const std::vector<Track>& tracks,
    const std::vector<DepartureEvent>& dep_events,
    const std::vector<DumpingEvent>& dump_events,
    uint32_t frame_id)
{
    uint64_t now = NowMs();

    // -- clusters 배열 --
    json j_clusters = json::array();
    for (size_t i = 0; i < clusters.size(); ++i) {
        const auto& cl = clusters[i];
        json j_cl;
        j_cl["id"]    = static_cast<int>(i);
        j_cl["x"]     = std::round(cl.centroid_x_mm * 10.0) / 10.0;
        j_cl["y"]     = std::round(cl.centroid_y_mm * 10.0) / 10.0;
        // j_cl["count"] = static_cast<int>(cl.points.size());
        j_cl["type"]  = ResolveType(cl, tracks);
        j_clusters.push_back(std::move(j_cl));
    }

    // -- tracks 배열 (사람 이동 데이터) --
    json j_tracks = json::array();
    for (const auto& tr : tracks) {
        json j_tr;
        j_tr["id"]    = tr.id;
        j_tr["x"]     = std::round(tr.x_mm * 10.0) / 10.0;
        j_tr["y"]     = std::round(tr.y_mm * 10.0) / 10.0;

        const char* state_str = "unknown";
        switch (tr.state) {
            case TrackState::Moving:     state_str = "moving"; break;
            case TrackState::Stationary: state_str = "stationary"; break;
            case TrackState::Departed:   state_str = "departed"; break;
            case TrackState::Lost:       state_str = "lost"; break;
        }
        j_tr["state"]          = state_str;
        // j_tr["vx"]             = std::round(tr.vx_mm * 10.0) / 10.0;
        // j_tr["vy"]             = std::round(tr.vy_mm * 10.0) / 10.0;
        j_tr["is_dumped_item"] = tr.is_dumped_item;
        j_tr["is_dump_suspect"]= tr.is_dump_suspect;

        j_tracks.push_back(std::move(j_tr));
    }

    // -- departure_event (이탈 이벤트) --
    json j_dep_event = nullptr;
    if (!dep_events.empty()) {
        const auto& evt = dep_events.back();
        j_dep_event = {
            {"type",      "departed"},
            {"track_id",  static_cast<int>(evt.track_id)},
            {"x",         std::round(evt.x_mm * 10.0) / 10.0},
            {"y",         std::round(evt.y_mm * 10.0) / 10.0},
            {"timestamp", MsToIso8601(evt.timestamp_ms)}
        };
    }

    // -- dumping_event (투기 확정 이벤트) --
    json j_dump_event = nullptr;
    if (!dump_events.empty()) {
        const auto& evt = dump_events.back();
        j_dump_event = {
            {"type",      "dumping_confirmed"},
            {"person_id", static_cast<int>(evt.person_track_id)},
            {"person_x",  std::round(evt.person_x_mm * 10.0) / 10.0},
            {"person_y",  std::round(evt.person_y_mm * 10.0) / 10.0},
            {"object_id", static_cast<int>(evt.object_track_id)},
            {"object_x",  std::round(evt.object_x_mm * 10.0) / 10.0},
            {"object_y",  std::round(evt.object_y_mm * 10.0) / 10.0},
            {"timestamp", MsToIso8601(evt.timestamp_ms)}
        };
    }

    // -- 최종 패킷 조립 (기존 형식 주석 처리) --
    /*
    json packet;
    packet["frame_id"]        = frame_id;
    packet["timestamp"]       = MsToIso8601(now);
    packet["clusters"]        = std::move(j_clusters);
    packet["tracks"]          = std::move(j_tracks);
    packet["departure_event"] = std::move(j_dep_event);
    packet["dumping_event"]   = std::move(j_dump_event);
    */

    // -- 새 형식: {"type": "FRAME", "objects": [...]} --
    json packet;
    packet["type"]    = "FRAME";
    packet["objects"] = std::move(j_clusters);

    return packet.dump();
}

// ── Send: 직렬화 + 크기 검증 + UDP 전송 ────────────────────────────
//
//   65507 bytes 초과 시:
//     1. 클러스터를 포인트 수 내림차순 정렬
//     2. 뒤에서부터 하나씩 제거하며 크기 확인
//     3. 그래도 초과하면 전송 포기
//
bool JsonPacketSender::Send(const std::vector<Cluster>& clusters,
                            const std::vector<Track>& tracks,
                            const std::vector<DepartureEvent>& dep_events,
                            const std::vector<DumpingEvent>& dump_events,
                            uint32_t frame_id)
{
    // 1) 전체 직렬화 시도
    std::string payload = Serialize(clusters, tracks, dep_events, dump_events, frame_id);

    // 2) 크기 제한 체크
    if (payload.size() <= UDP_MAX_DGRAM) {
        if (udp_.SendBuffer(payload.data(), payload.size())) {
            sent_count_++;
            return true;
        }
        drop_count_++;
        return false;
    }

    // 3) 초과 -> 클러스터 수를 줄여서 재직렬화
    std::vector<Cluster> trimmed = clusters;
    std::sort(trimmed.begin(), trimmed.end(),
              [](const Cluster& a, const Cluster& b) {
                  return a.points.size() > b.points.size();
              });

    while (!trimmed.empty()) {
        trimmed.pop_back();
        trunc_count_++;

        payload = Serialize(trimmed, tracks, dep_events, dump_events, frame_id);

        if (payload.size() <= UDP_MAX_DGRAM) {
            std::fprintf(stderr,
                "[JSON] Truncated to %zu clusters (payload %zu bytes)\n",
                trimmed.size(), payload.size());

            if (udp_.SendBuffer(payload.data(), payload.size())) {
                sent_count_++;
                return true;
            }
            drop_count_++;
            return false;
        }
    }

    std::fprintf(stderr, "[JSON] Cannot fit packet under 65507 bytes\n");
    drop_count_++;
    return false;
}

} // namespace ld19
