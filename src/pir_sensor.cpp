/*
 * Copyright (c) 2026 김승연
 *
 * This software is released under the MIT License.
 * See LICENSE file in the project root for details.
 *
 * Project: 데이터 무결성 보증형 디지털 트윈 관제 플랫폼
 * Module : EMBEDDED - PIR 센서 인터페이스 (GPIO sysfs)
 */

/**
 * @file  pir_sensor.cpp
 * @brief PIR 센서 GPIO 폴링 구현 (Linux sysfs)
 * @date  2026
 *
 * Linux sysfs GPIO 인터페이스:
 *   /sys/class/gpio/export         → 핀 번호 기록하여 활성화
 *   /sys/class/gpio/gpioN/direction → "in" 기록하여 입력 모드 설정
 *   /sys/class/gpio/gpioN/value    → '0' 또는 '1' 읽기
 *
 * 비-Linux 환경:
 *   모든 함수가 안전하게 동작하며, IsMotionDetected()는 항상 true 반환.
 */

#include "pir_sensor.h"

#include <cstdio>

#ifdef __linux__
#include <fstream>
#include <sstream>
#include <unistd.h>
#endif

namespace ld19 {

// ── 생성자 / 소멸자 ─────────────────────────────────────────────────
PirSensor::PirSensor(const PirSensorConfig& config)
    : config_(config) {}

PirSensor::~PirSensor() {
#ifdef __linux__
    // GPIO unexport (정리)
    if (initialized_) {
        std::ofstream unexport_fs("/sys/class/gpio/unexport");
        if (unexport_fs.is_open()) {
            unexport_fs << config_.gpio_pin;
        }
    }
#endif
}

// ── 초기화 ──────────────────────────────────────────────────────────
bool PirSensor::Init() {
#ifdef __linux__
    // sysfs 경로 구성
    std::ostringstream oss;
    oss << "/sys/class/gpio/gpio" << config_.gpio_pin;
    std::string gpio_dir = oss.str();
    value_path_ = gpio_dir + "/value";

    // 1) GPIO export (이미 export 되어 있을 수 있음)
    {
        std::ifstream check(gpio_dir + "/direction");
        if (!check.is_open()) {
            std::ofstream export_fs("/sys/class/gpio/export");
            if (!export_fs.is_open()) {
                std::fprintf(stderr,
                    "[PIR] ERROR: Cannot open /sys/class/gpio/export. "
                    "Run with sudo or add user to gpio group.\n");
                return false;
            }
            export_fs << config_.gpio_pin;
            export_fs.close();
            // export 후 sysfs 파일 생성 대기
            usleep(100000);  // 100ms
        }
    }

    // 2) direction = "in"
    {
        std::ofstream dir_fs(gpio_dir + "/direction");
        if (!dir_fs.is_open()) {
            std::fprintf(stderr,
                "[PIR] ERROR: Cannot set direction for GPIO %u\n",
                config_.gpio_pin);
            return false;
        }
        dir_fs << "in";
    }

    // 3) 초기 읽기 테스트
    if (!ReadGpioRaw()) {
        // 읽기 실패해도 계속 진행 (첫 프레임에 복구될 수 있음)
        std::fprintf(stderr,
            "[PIR] WARNING: Initial read failed for GPIO %u\n",
            config_.gpio_pin);
    }

    initialized_ = true;
    std::printf("[PIR] Initialized on GPIO %u (active_%s, debounce=%u, holdoff=%u)\n",
                config_.gpio_pin,
                config_.active_high ? "HIGH" : "LOW",
                config_.debounce_frames,
                config_.holdoff_frames);
    return true;

#else
    // 비-Linux: 항상 초기화 성공, 항상 모션 감지
    initialized_   = true;
    motion_active_ = true;
    std::printf("[PIR] Non-Linux platform — PIR disabled, always returns motion=ON\n");
    return true;
#endif
}

// ── GPIO 원시 값 읽기 ───────────────────────────────────────────────
bool PirSensor::ReadGpioRaw() {
#ifdef __linux__
    std::ifstream val_fs(value_path_);
    if (!val_fs.is_open()) {
        return false;
    }

    char ch = '0';
    val_fs >> ch;

    bool pin_high = (ch == '1');
    raw_state_ = config_.active_high ? pin_high : !pin_high;
    return true;
#else
    raw_state_ = true;
    return true;
#endif
}

// ── 프레임 단위 읽기 (디바운스 + 홀드오프 적용) ─────────────────────
void PirSensor::Read() {
    state_changed_ = false;

#ifndef __linux__
    motion_active_ = true;
    return;
#else
    if (!initialized_) {
        motion_active_ = true;
        return;
    }

    // GPIO 원시 값 읽기
    if (!ReadGpioRaw()) {
        // 읽기 실패 시 이전 상태 유지
        return;
    }

    bool prev_active = motion_active_;

    // 홀드오프 카운터 갱신
    if (holdoff_remaining_ > 0) {
        holdoff_remaining_--;
    }

    // 디바운스 로직: 원시 상태가 현재 활성 상태와 다르면 카운트 시작
    if (raw_state_ != motion_active_) {
        stable_count_++;

        if (stable_count_ >= config_.debounce_frames) {
            // 상태 전환
            if (raw_state_) {
                // OFF -> ON 전환
                motion_active_    = true;
                holdoff_remaining_ = config_.holdoff_frames;
            } else {
                // ON -> OFF 전환 (홀드오프 만료 후에만)
                if (holdoff_remaining_ == 0) {
                    motion_active_ = false;
                }
            }
            stable_count_ = 0;
        }
    } else {
        stable_count_ = 0;
    }

    // 모션 감지 시 홀드오프 갱신 (센서가 계속 HIGH면 유지)
    if (raw_state_ && motion_active_) {
        holdoff_remaining_ = config_.holdoff_frames;
    }

    state_changed_ = (motion_active_ != prev_active);

    if (state_changed_) {
        std::printf("[PIR] Motion: %s\n",
                    motion_active_ ? "ON" : "OFF");
    }
#endif
}

} // namespace ld19
