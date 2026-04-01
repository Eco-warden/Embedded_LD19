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
 * @file  ld19_lidar.cpp
 * @brief LD19 LiDAR 래퍼 클래스 구현
 * @date  2026
 */

#include "ld19_lidar.h"

#include <chrono>
#include <cstdio>

namespace ld19 {

// ── 에러 코드 → 문자열 변환 ─────────────────────────────────────────
const char* ErrorToString(Error e) {
    switch (e) {
        case Error::None:             return "None";
        case Error::ConnectionFailed: return "ConnectionFailed: serial port open or comm failed";
        case Error::CommTimeout:      return "CommTimeout: no response from LiDAR within timeout";
        case Error::ScanTimeout:      return "ScanTimeout: scan data not received within timeout";
        case Error::AlreadyRunning:   return "AlreadyRunning: Start() called while already scanning";
        case Error::NotRunning:       return "NotRunning: sensor is not started";
    }
    return "Unknown";
}

// ── 나노초 타임스탬프 제공 ──────────────────────────────────────────
uint64_t LD19Lidar::GetTimestampNs() {
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<nanoseconds>(
            steady_clock::now().time_since_epoch()
        ).count()
    );
}

// ── 생성자 / 소멸자 ─────────────────────────────────────────────────
LD19Lidar::LD19Lidar() = default;

LD19Lidar::~LD19Lidar() {
    if (running_) {
        Stop();
    }
}

// ── Start ────────────────────────────────────────────────────────────
Error LD19Lidar::Start(const Config& cfg) {
    if (running_) {
        return Error::AlreadyRunning;
    }

    cfg_ = cfg;

    driver_ = new ldlidar::LDLidarDriver();

    // 타임스탬프 콜백 등록
    driver_->RegisterGetTimestampFunctional(&LD19Lidar::GetTimestampNs);

    // 노이즈 필터 설정
    driver_->EnableFilterAlgorithnmProcess(cfg_.enable_filter);

    // 시리얼 포트를 통해 LD19 시작
    bool ok = driver_->Start(
        ldlidar::LDType::LD_19,
        cfg_.serial_port,
        cfg_.baudrate,
        ldlidar::COMM_SERIAL_MODE
    );
    if (!ok) {
        delete driver_;
        driver_ = nullptr;
        return Error::ConnectionFailed;
    }

    // 통신 연결 대기
    if (!driver_->WaitLidarCommConnect(cfg_.connect_timeout_ms)) {
        driver_->Stop();
        delete driver_;
        driver_ = nullptr;
        return Error::CommTimeout;
    }

    running_ = true;
    return Error::None;
}

// ── Stop ─────────────────────────────────────────────────────────────
Error LD19Lidar::Stop() {
    if (!running_ || !driver_) {
        return Error::NotRunning;
    }

    driver_->Stop();
    delete driver_;
    driver_  = nullptr;
    running_ = false;
    return Error::None;
}

// ── GetScanFrame ─────────────────────────────────────────────────────
Error LD19Lidar::GetScanFrame(ScanFrame& frame) {
    if (!running_ || !driver_) {
        return Error::NotRunning;
    }

    frame.clear();

    ldlidar::Points2D raw_points;
    ldlidar::LidarStatus status =
        driver_->GetLaserScanData(raw_points, cfg_.scan_timeout_ms);

    switch (status) {
        case ldlidar::LidarStatus::NORMAL: {
            frame.reserve(raw_points.size());
            for (const auto& pt : raw_points) {
                frame.push_back(ScanPoint{
                    .angle_deg   = pt.angle,
                    .distance_mm = pt.distance,
                    .intensity   = pt.intensity,
                });
            }
            return Error::None;
        }
        case ldlidar::LidarStatus::DATA_TIME_OUT:
            return Error::ScanTimeout;

        case ldlidar::LidarStatus::DATA_WAIT:
            // 아직 데이터 수집 중 — 빈 프레임 반환, 에러 아님
            return Error::None;

        default:
            return Error::ScanTimeout;
    }
    
    return Error::ScanTimeout;
}

// ── GetScanFrequency ─────────────────────────────────────────────────
bool LD19Lidar::GetScanFrequency(double& hz) {
    if (!running_ || !driver_) {
        return false;
    }
    return driver_->GetLidarScanFreq(hz);
}

// ── IsRunning ────────────────────────────────────────────────────────
bool LD19Lidar::IsRunning() const {
    return running_;
}

} // namespace ld19
