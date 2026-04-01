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
 * @file  ld19_lidar.h
 * @brief LD19 LiDAR 센서 래퍼 클래스
 * @date  2026
 *
 * LDROBOT ldlidar_stl_sdk 기반 LD19 제어 인터페이스.
 * 연속 스캔 데이터를 (각도, 거리, 강도) 벡터로 반환한다.
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <functional>
#include <atomic>

#include "core/ldlidar_driver.h"

namespace ld19 {

// ── 스캔 포인트 구조체 ──────────────────────────────────────────────
struct ScanPoint {
    float    angle_deg;   // 각도 (0 ~ 360, degree)
    uint16_t distance_mm; // 거리 (mm)
    uint8_t  intensity;   // 반사 강도 (0-255)
};

using ScanFrame = std::vector<ScanPoint>;

// ── 에러 코드 ────────────────────────────────────────────────────────
enum class Error {
    None,
    ConnectionFailed,  // 시리얼 포트 연결 실패
    CommTimeout,       // 통신 대기 타임아웃
    ScanTimeout,       // 스캔 데이터 수신 타임아웃
    AlreadyRunning,    // 이미 스캔 중
    NotRunning,        // 스캔이 시작되지 않음
};

const char* ErrorToString(Error e);

// ── 설정 구조체 ──────────────────────────────────────────────────────
struct Config {
    std::string serial_port   = "/dev/ttyAMA0";
    uint32_t    baudrate      = 230400;
    int64_t     connect_timeout_ms = 3500;  // 연결 대기 타임아웃 (ms)
    int64_t     scan_timeout_ms    = 1500;  // 스캔 데이터 대기 타임아웃 (ms)
    bool        enable_filter = true;       // 노이즈 필터 활성화
};

// ── LD19 LiDAR 래퍼 클래스 ───────────────────────────────────────────
class LD19Lidar {
public:
    LD19Lidar();
    ~LD19Lidar();

    LD19Lidar(const LD19Lidar&) = delete;
    LD19Lidar& operator=(const LD19Lidar&) = delete;

    /**
     * @brief 센서에 연결하고 연속 스캔을 시작한다.
     * @param cfg  설정 (기본값 사용 가능)
     * @return Error::None 성공, 그 외 에러 코드
     */
    Error Start(const Config& cfg = Config{});

    /**
     * @brief 스캔을 중지하고 시리얼 포트를 닫는다.
     * @return Error::None 성공
     */
    Error Stop();

    /**
     * @brief 한 프레임(360° 1회전)의 스캔 데이터를 가져온다.
     * @param[out] frame  스캔 포인트 벡터
     * @return Error::None 성공, Error::ScanTimeout 시간 초과
     */
    Error GetScanFrame(ScanFrame& frame);

    /**
     * @brief 현재 스캔 주파수(Hz)를 반환한다.
     */
    bool GetScanFrequency(double& hz);

    /**
     * @brief 센서가 정상 동작 중인지 확인한다.
     */
    bool IsRunning() const;

private:
    ldlidar::LDLidarDriver* driver_ = nullptr;
    std::atomic<bool>       running_{false};
    Config                  cfg_;

    static uint64_t GetTimestampNs();
};

} // namespace ld19
