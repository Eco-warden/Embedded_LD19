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
 * @file  event_notifier.cpp
 * @brief 비동기 HTTP POST 이벤트 전송 + JSONL 파일 큐 + 백그라운드 재전송
 * @date  2026
 */

#include "event_notifier.h"

#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <sstream>
#include <algorithm>

#include <curl/curl.h>

namespace ld19 {

// ── curl 응답 콜백 (본문 무시) ──────────────────────────────────────
static size_t CurlWriteDiscard(void* /*ptr*/, size_t size, size_t nmemb,
                               void* /*userdata*/) {
    return size * nmemb;
}

// ── curl_global_init 중복 방지 (프로세스 전체에서 1회만) ─────────────
static std::once_flag s_curl_init_flag;

// ── 생성자 / 소멸자 ─────────────────────────────────────────────────
EventNotifier::EventNotifier(const NotifierConfig& cfg) : cfg_(cfg) {
    std::call_once(s_curl_init_flag, []() {
        curl_global_init(CURL_GLOBAL_DEFAULT);
    });
}

EventNotifier::~EventNotifier() {
    StopRetryThread();
    // curl_global_cleanup은 프로세스 종료 시 자동 정리에 맡김
    // (여러 EventNotifier 인스턴스 생성 가능성 대비)
}

// ── epoch ms → ISO 8601 ─────────────────────────────────────────────
std::string EventNotifier::MsToIso8601(uint64_t epoch_ms) {
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

// ── JSON 문자열 이스케이프 ──────────────────────────────────────────
static std::string EscapeJson(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 8);
    for (char c : s) {
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default:   out += c;      break;
        }
    }
    return out;
}

// ── DepartureEvent → JSON ───────────────────────────────────────────
std::string EventNotifier::ToJson(const DepartureEvent& evt) {
    char buf[512];
    std::snprintf(buf, sizeof(buf),
        "{"
            "\"timestamp\":\"%s\","
            "\"x\":%.1f,"
            "\"y\":%.1f,"
            "\"cluster_id\":%u,"
            "\"type\":\"abandoned\""
        "}",
        EscapeJson(MsToIso8601(evt.timestamp_ms)).c_str(),
        evt.x_mm,
        evt.y_mm,
        evt.track_id
    );
    return buf;
}

// ── DumpingEvent → JSON ─────────────────────────────────────────────
std::string EventNotifier::DumpingToJson(const DumpingEvent& evt) {
    char buf[1024];
    std::snprintf(buf, sizeof(buf),
        "{"
            "\"type\":\"dumping\","
            "\"timestamp\":\"%s\","
            "\"person_id\":%u,"
            "\"person_x\":%.1f,"
            "\"person_y\":%.1f,"
            "\"person_cumulative_dist\":%.1f,"
            "\"object_id\":%u,"
            "\"object_x\":%.1f,"
            "\"object_y\":%.1f"
        "}",
        EscapeJson(MsToIso8601(evt.timestamp_ms)).c_str(),
        evt.person_track_id,
        evt.person_x_mm,
        evt.person_y_mm,
        evt.person_cumulative_dist_mm,
        evt.object_track_id,
        evt.object_x_mm,
        evt.object_y_mm
    );
    return buf;
}

// ── libcurl HTTP POST ───────────────────────────────────────────────
bool EventNotifier::HttpPost(const std::string& json_body) {
    CURL* curl = curl_easy_init();
    if (!curl) {
        std::fprintf(stderr, "[NOTIFIER] curl_easy_init() failed\n");
        return false;
    }

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    headers = curl_slist_append(headers, "Accept: application/json");

    curl_easy_setopt(curl, CURLOPT_URL, cfg_.endpoint_url.c_str());
    curl_easy_setopt(curl, CURLOPT_POST, 1L);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_body.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE,
                     static_cast<long>(json_body.size()));
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, cfg_.timeout_ms);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, cfg_.timeout_ms);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, CurlWriteDiscard);
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);

    CURLcode res = curl_easy_perform(curl);

    bool success = false;
    if (res == CURLE_OK) {
        long http_code = 0;
        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
        success = (http_code >= 200 && http_code < 300);
        if (!success) {
            std::fprintf(stderr, "[NOTIFIER] HTTP %ld from %s\n",
                         http_code, cfg_.endpoint_url.c_str());
        }
    } else {
        std::fprintf(stderr, "[NOTIFIER] curl error: %s\n",
                     curl_easy_strerror(res));
    }

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);

    return success;
}

// ── EnqueueJson: 비동기 전송 큐에 추가 (논블로킹) ──────────────────
void EventNotifier::EnqueueJson(const std::string& json) {
    {
        std::lock_guard<std::mutex> lock(send_queue_mutex_);
        send_queue_.push(json);
    }
    send_queue_cv_.notify_one();
}

// ── Send: 비동기 큐에 넣기 (논블로킹) ──────────────────────────────
void EventNotifier::Send(const DepartureEvent& evt) {
    EnqueueJson(ToJson(evt));
}

void EventNotifier::SendDumping(const DumpingEvent& evt) {
    EnqueueJson(DumpingToJson(evt));
}

// ── SendLoop: 전송 전용 스레드 루프 ─────────────────────────────────
void EventNotifier::SendLoop() {
    while (send_running_) {
        std::string json;

        {
            std::unique_lock<std::mutex> lock(send_queue_mutex_);
            send_queue_cv_.wait(lock, [this] {
                return !send_queue_.empty() || !send_running_;
            });

            if (!send_running_ && send_queue_.empty()) break;
            if (send_queue_.empty()) continue;

            json = std::move(send_queue_.front());
            send_queue_.pop();
        }

        if (HttpPost(json)) {
            sent_count_++;
        } else {
            fail_count_++;
            {
                std::lock_guard<std::mutex> lock(fail_queue_mutex_);
                pending_queue_.push(json);
            }
            AppendToFile(json);
        }
    }

    // 종료 시 큐에 남은 것 모두 처리
    std::lock_guard<std::mutex> lock(send_queue_mutex_);
    while (!send_queue_.empty()) {
        const std::string& json = send_queue_.front();
        if (HttpPost(json)) {
            sent_count_++;
        } else {
            fail_count_++;
            std::lock_guard<std::mutex> flock(fail_queue_mutex_);
            pending_queue_.push(json);
            AppendToFile(json);
        }
        send_queue_.pop();
    }
}

// ── 로컬 파일 큐: 추가 ─────────────────────────────────────────────
void EventNotifier::AppendToFile(const std::string& json_line) {
    std::ofstream ofs(cfg_.queue_file_path, std::ios::app);
    if (ofs.is_open()) {
        ofs << json_line << "\n";
    } else {
        std::fprintf(stderr, "[NOTIFIER] Cannot open queue file: %s\n",
                     cfg_.queue_file_path.c_str());
    }
}

// ── 로컬 파일 큐: 읽기 ─────────────────────────────────────────────
std::vector<std::string> EventNotifier::ReadQueueFile() {
    std::vector<std::string> lines;
    std::ifstream ifs(cfg_.queue_file_path);
    if (!ifs.is_open()) return lines;

    std::string line;
    while (std::getline(ifs, line)) {
        if (!line.empty()) {
            lines.push_back(line);
        }
    }
    return lines;
}

// ── 로컬 파일 큐: 덮어쓰기 (남은 라인만 보존) ─────────────────────
void EventNotifier::WriteQueueFile(const std::vector<std::string>& lines) {
    std::ofstream ofs(cfg_.queue_file_path, std::ios::trunc);
    if (!ofs.is_open()) return;

    for (const auto& l : lines) {
        ofs << l << "\n";
    }
}

// ── FlushQueue: 파일 + 메모리 큐 일괄 재전송 ───────────────────────
size_t EventNotifier::FlushQueue() {
    std::vector<std::string> file_lines = ReadQueueFile();

    {
        std::lock_guard<std::mutex> lock(fail_queue_mutex_);
        while (!pending_queue_.empty()) {
            file_lines.push_back(pending_queue_.front());
            pending_queue_.pop();
        }
    }

    if (file_lines.empty()) return 0;

    std::sort(file_lines.begin(), file_lines.end());
    file_lines.erase(std::unique(file_lines.begin(), file_lines.end()),
                     file_lines.end());

    std::vector<std::string> still_failed;
    size_t success_count = 0;

    for (const auto& json : file_lines) {
        bool ok = false;
        for (uint32_t attempt = 0; attempt < cfg_.max_retries; ++attempt) {
            if (HttpPost(json)) {
                ok = true;
                break;
            }
        }

        if (ok) {
            success_count++;
            sent_count_++;
        } else {
            still_failed.push_back(json);
        }
    }

    WriteQueueFile(still_failed);

    if (!still_failed.empty()) {
        std::fprintf(stderr, "[NOTIFIER] %zu events still queued after flush\n",
                     still_failed.size());
    }

    return success_count;
}

// ── GetQueueSize ────────────────────────────────────────────────────
size_t EventNotifier::GetQueueSize() const {
    std::lock_guard<std::mutex> lock(fail_queue_mutex_);
    return pending_queue_.size();
}

// ── 스레드 시작/중지 ────────────────────────────────────────────────
void EventNotifier::StartRetryThread() {
    // 전송 전용 스레드 시작
    if (!send_running_) {
        send_running_ = true;
        send_thread_ = std::thread(&EventNotifier::SendLoop, this);
    }

    // 재전송 스레드 시작
    if (!retry_running_) {
        retry_running_ = true;
        retry_thread_ = std::thread(&EventNotifier::RetryLoop, this);
    }
}

void EventNotifier::StopRetryThread() {
    // 전송 스레드 중지
    if (send_running_) {
        send_running_ = false;
        send_queue_cv_.notify_all();
        if (send_thread_.joinable()) {
            send_thread_.join();
        }
    }

    // 재전송 스레드 중지
    if (retry_running_) {
        retry_running_ = false;
        retry_cv_.notify_all();
        if (retry_thread_.joinable()) {
            retry_thread_.join();
        }
    }
}

void EventNotifier::RetryLoop() {
    while (retry_running_) {
        {
            std::unique_lock<std::mutex> lock(retry_cv_mutex_);
            retry_cv_.wait_for(lock,
                std::chrono::seconds(cfg_.retry_interval_sec),
                [this]{ return !retry_running_.load(); }
            );
        }

        if (!retry_running_) break;

        size_t flushed = FlushQueue();
        if (flushed > 0) {
            std::printf("[NOTIFIER] Background flush: %zu events resent\n",
                        flushed);
        }
    }
}

} // namespace ld19
