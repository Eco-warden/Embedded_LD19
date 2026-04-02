# 프로젝트 상태 보고서 (PROJECT_STATE.md)

이 문서는 **데이터 무결성 보증형 디지털 트윈 관제 플랫폼**의 임베디드 모듈(Embedded_LD19)의 현재 구현 상태와 핵심 로직을 요약한 문서입니다.

## 1. 프로젝트 개요
- **목적**: LD19 LiDAR 센서를 이용하여 불법 투기를 실시간 탐지하고, 탐지된 데이터를 Unity(시각화)와 FastAPI(데이터 보존)로 전송하는 시스템.
- **핵심 기술**: C++17, LD19 LiDAR, DBSCAN 클러스터링, 칼만 필터(추적), libcurl(HTTP), POSIX 소켓(UDP).

## 2. 데이터 전송 규격 (현재 설정)

### A. Unity (UDP 실시간 전송)
- **전송 빈도**: 매 프레임 (약 10Hz)
- **전송 형식**: JSON (`{"type": "FRAME", "objects": [...]}`)
- **주요 변경 사항**:
    - `objects` 리스트 내에 각 객체의 `id`, `x`, `y`, `type` 정보 포함.
    - **주석 처리됨**: 객체의 크기(`count`), 속도(`vx`, `vy`) 필드는 전송 효율을 위해 코드상에서 주석 처리됨.
    - **목적**: 실시간 점구름 및 객체 이동 데이터를 Unity 디지털 트윈에 동기화.

### B. FastAPI (HTTP POST 이벤트 전송)
- **전송 주소**: `https://api.ecowarden.systems/api/dumping-event`
- **전송 조건**: **쓰레기 투기(Dumping) 이벤트 확정 시**에만 1회 전송.
- **데이터 내용**: 투기자(사람) 위치, 투기물 위치, 누적 이동 거리, (선택적) 카메라 캡처 이미지.
- **특이 사항**: 전송 실패 시 `/tmp/ld19_event_queue.jsonl`에 자동 저장 후 백그라운드에서 재전송 시도.

## 3. 주요 소스코드 역할
- `src/main.cpp`: 프로그램의 메인 루프. LiDAR 초기화, 데이터 처리 파이프라인 제어.
- `src/json_packet.cpp`: Unity용 JSON 패킷 생성 및 전송 로직 (현재 `objects` 형식으로 최적화됨).
- `src/event_notifier.cpp`: FastAPI 서버로의 HTTP 이벤트 전송 및 실패 시 큐잉 로직.
- `src/cluster_tracker.cpp`: 객체 탐지(DBSCAN) 및 실시간 객체 추적, 투기 판정 로직.
- `src/ld19_lidar.cpp`: LD19 LiDAR 센서와의 시리얼 통신 인터페이스.
- `src/udp_sender.cpp`: 저수준 UDP 소켓 관리.

## 4. 실행 및 설정 정보
- **기본 API 주소**: `https://api.ecowarden.systems/api/dumping-event` (운영 서버 주소로 고정됨).
- **Unity 기본 주소**: 실행 시 인자가 없을 경우 `192.168.20.52:9090`를 시도하며, 실제 Unity PC IP를 인자로 주는 것을 권장.
- **실행 예시**:
  ```bash
  sudo ./build/ld19_lidar_app /dev/ttyAMA0 https://api.ecowarden.systems/api/dumping-event 192.168.20.52:9090
  ```


## 5. 트러블슈팅 및 환경 설정 (중요)

### A. aarch64(ARM64) 환경 빌드 이슈 해결
- **문제**: `__SANE_USERSPACE_TYPES__` 매크로 중복 정의 및 `uint64_t` 타입 불일치(`unsigned long` vs `unsigned long long`)로 인한 컴파일 에러 발생.
- **원인**: 리눅스 커널 헤더와 유저 공간 헤더 간의 타입 정의 방식 충돌.
- **해결 방법**:
1.  **CMake 전역 정의**: `CMakeLists.txt`에 `add_compile_options(-D__SANE_USERSPACE_TYPES__)`를 추가하여 빌드 시스템 차원에서 일관성 유지.
2.  **소스 코드 최상단 매크로 선언**: `src/main.cpp`, `src/sim_main.cpp` 등 주요 진입점 파일 최상단에 `#ifndef __SANE_USERSPACE_TYPES__` 트릭을 적용하여 헤더 포함 전 매크로 확정.
3.  **헤더 포함 순서 강제 조정**: 표준 C++ 헤더(`<cstdint>`)를 시스템/커널 헤더(`<csignal>`, `<unistd.h>` 등)보다 항상 먼저 포함하도록 조정하여 `uint64_t` 타입 정의를 선점.
### B. 빌드 및 실행 권장 사양
- **OS**: Linux (Ubuntu 20.04+ 권장, aarch64/x86_64)
- **도구**: CMake 3.16+, GCC/G++ 9+
- **빌드 명령어**: `cd build && cmake .. && make -j$(nproc)`

---
**최종 업데이트 시각**: 2026-04-02 (KST)
