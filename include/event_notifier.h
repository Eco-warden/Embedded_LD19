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
 * @file  event_notifier.h
 * @brief 이탈/투기 이벤트를 FastAPI 서버로 HTTP POST 전송 (비동기) + 실패 시 로컬 파일 큐잉
 * @date  2026
 *
 * 흐름:
 *   Send() / SendDumping()
 *       │
 *       ▼
 *   send_queue_ (비동기 큐)
 *       │
 *       ▼ (전송 스레드)
 *   HttpPost()
 *       │
 *       ├─ 성공 → 완료
 *       └─ 실패 → pending_queue_ + 로컬 파일
 *                  │
 *                  ▼ (재전송 스레드)
 *              FlushQueue()
 */

#pragma once

#include "cluster_tracker.h"

#include <string>
#include <vector>
#include <queue>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

namespace ld19 {

// ── 전송 설정 ────────────────────────────────────────────────────────
struct NotifierConfig {
    std::string endpoint_url    = "http://localhost:8000/api/v1/events";
    std::string queue_file_path = "/tmp/ld19_event_queue.jsonl";
    long        timeout_ms      = 3000;        // HTTP 타임아웃 (ms)
    uint32_t    max_retries     = 3;           // 큐 재전송 시 최대 재시도 횟수
    uint32_t    retry_interval_sec = 10;       // 큐 재전송 주기 (초)
};

// ── 이벤트 전송기 ────────────────────────────────────────────────────
class EventNotifier {
public:
    explicit EventNotifier(const NotifierConfig& cfg = NotifierConfig{});
    ~EventNotifier();

    EventNotifier(const EventNotifier&) = delete;
    EventNotifier& operator=(const EventNotifier&) = delete;

    /**
     * @brief 이탈 이벤트를 비동기 전송 큐에 넣는다 (논블로킹).
     */
    void Send(const DepartureEvent& evt);

    /**
     * @brief 투기 이벤트를 비동기 전송 큐에 넣는다 (논블로킹).
     */
    void SendDumping(const DumpingEvent& evt);

    /**
     * @brief 큐 파일에 쌓인 이벤트를 모두 재전송한다.
     * @return 성공적으로 재전송된 이벤트 수
     */
    size_t FlushQueue();

    /**
     * @brief 전송 스레드 + 재전송 스레드를 시작한다.
     */
    void StartRetryThread();

    /**
     * @brief 모든 스레드를 중지한다.
     */
    void StopRetryThread();

    size_t GetQueueSize() const;
    size_t GetSentCount() const { return sent_count_; }
    size_t GetFailCount() const { return fail_count_; }

private:
    NotifierConfig cfg_;

    std::atomic<size_t> sent_count_{0};
    std::atomic<size_t> fail_count_{0};

    // ── 비동기 전송 큐 (Send 호출 → 전송 스레드가 처리) ────────────
    mutable std::mutex          send_queue_mutex_;
    std::queue<std::string>     send_queue_;
    std::condition_variable     send_queue_cv_;
    std::thread                 send_thread_;
    std::atomic<bool>           send_running_{false};

    // ── 실패 이벤트 큐 (전송 실패 → 파일+메모리 큐 → 재전송 스레드) ─
    mutable std::mutex          fail_queue_mutex_;
    std::queue<std::string>     pending_queue_;

    // ── 백그라운드 재전송 스레드 ────────────────────────────────────
    std::thread                 retry_thread_;
    std::atomic<bool>           retry_running_{false};
    std::mutex                  retry_cv_mutex_;
    std::condition_variable     retry_cv_;

    static std::string ToJson(const DepartureEvent& evt);
    static std::string DumpingToJson(const DumpingEvent& evt);
    static std::string MsToIso8601(uint64_t epoch_ms);

    bool HttpPost(const std::string& json_body);
    void EnqueueJson(const std::string& json);
    void SendLoop();
    void AppendToFile(const std::string& json_line);
    std::vector<std::string> ReadQueueFile();
    void WriteQueueFile(const std::vector<std::string>& lines);
    void RetryLoop();
};

} // namespace ld19
