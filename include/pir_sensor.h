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
 * @file  pir_sensor.h
 * @brief PIR(Passive Infrared) 센서 인터페이스
 * @date  2026
 *
 * 동작:
 *   - Linux sysfs GPIO를 통해 PIR 센서 출력 핀을 폴링
 *   - 디바운스 히스테리시스를 적용하여 안정적인 모션 상태 제공
 *   - 비-Linux 환경에서는 항상 모션 감지 상태를 반환 (빌드 호환성)
 *
 * 연결:
 *   PIR VCC → 5V, GND → GND, OUT → BCM GPIO (기본 17)
 */

#pragma once

#include <cstdint>
#include <string>

namespace ld19 {

// ── PIR 센서 설정 ────────────────────────────────────────────────────
struct PirSensorConfig {
    uint32_t gpio_pin          = 17;    // BCM GPIO 핀 번호
    bool     active_high       = true;  // true: HIGH=모션감지, false: LOW=모션감지
    uint32_t debounce_frames   = 3;     // 상태 전환에 필요한 연속 프레임 수
    uint32_t holdoff_frames    = 50;    // 모션 감지 후 최소 유지 프레임 (5초@10Hz)
};

// ── PIR 센서 클래스 ──────────────────────────────────────────────────
class PirSensor {
public:
    explicit PirSensor(const PirSensorConfig& config = PirSensorConfig{});
    ~PirSensor();

    // 복사/이동 금지 (파일 디스크립터 관리)
    PirSensor(const PirSensor&)            = delete;
    PirSensor& operator=(const PirSensor&) = delete;

    /**
     * @brief GPIO 핀을 초기화한다 (sysfs export + direction 설정).
     * @return true: 초기화 성공, false: 실패 (권한 부족 등)
     */
    bool Init();

    /**
     * @brief GPIO 핀 값을 읽고 디바운스를 적용한다.
     *        매 프레임(10Hz) 호출 권장.
     */
    void Read();

    /**
     * @brief 디바운스 적용된 모션 감지 상태를 반환한다.
     * @return true: 모션 감지 중, false: 정지 상태
     */
    bool IsMotionDetected() const { return motion_active_; }

    /**
     * @brief 초기화 성공 여부를 반환한다.
     * @return true: 정상 동작 중
     */
    bool IsReady() const { return initialized_; }

    /**
     * @brief 상태 변경 발생 여부 (Read() 호출 후 확인).
     *        로그 출력 등에 사용.
     * @return true: 이번 Read()에서 상태가 변경됨
     */
    bool HasStateChanged() const { return state_changed_; }

private:
    PirSensorConfig config_;
    bool            initialized_   = false;
    bool            motion_active_ = false;
    bool            state_changed_ = false;

    // 디바운스 상태
    bool            raw_state_     = false;
    uint32_t        stable_count_  = 0;

    // 홀드오프: 모션 감지 후 최소 유지
    uint32_t        holdoff_remaining_ = 0;

    // sysfs 경로
    std::string     value_path_;

    bool ReadGpioRaw();
};

} // namespace ld19
