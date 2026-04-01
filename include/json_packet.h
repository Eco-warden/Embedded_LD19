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
 * @file  json_packet.h
 * @brief 클러스터 + 이벤트 데이터를 nlohmann/json으로 직렬화하여 UDP 전송
 * @date  2026
 *
 * JSON 패킷 구조:
 * {
 *   "frame_id": 42,
 *   "timestamp": "2026-03-27T12:34:56.789Z",
 *   "clusters": [
 *     {
 *       "id": 0,
 *       "x": 1200.0,
 *       "y": 350.0,
 *       "count": 12,
 *       "type": "normal"        ← normal / abandoned
 *     }
 *   ],
 *   "event": null               ← 이벤트 없으면 null
 *   // 또는
 *   "event": {
 *     "type": "abandoned",
 *     "x": 800.0,
 *     "y": 200.0,
 *     "cluster_id": 1,
 *     "timestamp": "2026-03-27T12:34:56.789Z"
 *   }
 * }
 *
 * 최대 패킷 크기: 65507 bytes (UDP 최대 payload)
 * 초과 시 클러스터를 포인트 수 내림차순으로 정렬 후 잘라냄.
 */

#pragma once

#include "scan_processor.h"
#include "cluster_tracker.h"
#include "udp_sender.h"

#include <nlohmann/json.hpp>

#include <string>
#include <vector>
#include <cstdint>
#include <atomic>

namespace ld19 {

// ── UDP JSON 최대 크기 ──────────────────────────────────────────────
static constexpr size_t UDP_MAX_DGRAM = 65507;

// ── JSON 패킷 빌더 + 송신기 ────────────────────────────────────────
class JsonPacketSender {
public:
    explicit JsonPacketSender(UdpSender& udp);

    /**
     * @brief 클러스터 + (옵션) 이탈 이벤트를 JSON 직렬화 후 UDP 전송
     *
     * @param clusters    현재 프레임 클러스터 목록
     * @param tracks      현재 트랙 목록
     * @param dep_events  이번 프레임의 이탈 이벤트
     * @param dump_events 이번 프레임의 투기 확정 이벤트
     * @param frame_id    프레임 번호
     * @return true: 전송 성공
     */
    bool Send(const std::vector<Cluster>& clusters,
              const std::vector<Track>& tracks,
              const std::vector<DepartureEvent>& dep_events,
              const std::vector<DumpingEvent>& dump_events,
              uint32_t frame_id);

    /**
     * @brief 클러스터 + 이벤트를 JSON으로 직렬화만 수행 (전송 없이)
     *        테스트/디버그용
     */
    static std::string Serialize(
        const std::vector<Cluster>& clusters,
        const std::vector<Track>& tracks,
        const std::vector<DepartureEvent>& dep_events,
        const std::vector<DumpingEvent>& dump_events,
        uint32_t frame_id
    );

    size_t GetSentCount()    const { return sent_count_; }
    size_t GetDropCount()    const { return drop_count_; }
    size_t GetTruncCount()   const { return trunc_count_; }

private:
    UdpSender& udp_;

    std::atomic<size_t> sent_count_{0};
    std::atomic<size_t> drop_count_{0};    // 전송 실패
    std::atomic<size_t> trunc_count_{0};   // 크기 초과로 클러스터 잘림

    /**
     * @brief epoch ms → ISO 8601 문자열
     */
    static std::string MsToIso8601(uint64_t epoch_ms);

    /**
     * @brief 클러스터의 이벤트 타입을 판별
     *        트랙 상태가 Departed인 경우 "abandoned", 그 외 "normal"
     */
    static std::string ResolveType(
        const Cluster& cluster,
        const std::vector<Track>& tracks
    );

    static uint64_t NowMs();
};

} // namespace ld19
