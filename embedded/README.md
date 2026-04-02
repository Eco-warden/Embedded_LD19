# EMBEDDED — LD19 LiDAR 센서 인터페이스 및 객체 탐지 모듈

> **이 문서는 `embedded/` 폴더의 C++ 센서 코드에 대한 문서입니다.**
> 전체 프로젝트 README는 [루트 디렉토리](../README.md)를 참고하세요.

LD19 LiDAR로부터 2D 점구름을 수집하고, 전처리 → DBSCAN 클러스터링 → 객체 추적 → 이탈(투기) 탐지까지 수행한 뒤,
결과를 FastAPI 서버(HTTP)와 Unity 클라이언트(UDP)로 실시간 전송하는 임베디드 C++ 모듈입니다.

- **담당자**: 김승연 (팀장)
- **언어**: C++17
- **제작년도**: 2026

---

## 목차

- [하드웨어 구성](#하드웨어-구성)
- [폴더 구조](#폴더-구조)
- [데이터 흐름](#데이터-흐름)
- [핵심 파라미터](#핵심-파라미터)
- [의존성](#의존성)
- [빌드 방법](#빌드-방법)
- [실행 방법](#실행-방법)
- [전송 패킷 구조](#전송-패킷-구조)
- [트러블슈팅](#트러블슈팅)
- [라이선스](#라이선스)

---

## 하드웨어 구성

| 항목 | 사양 |
|---|---|
| 센서 | LD19 LiDAR (LDROBOT) |
| 연결 | 시리얼 `/dev/ttyUSB0`, baudrate 230400 |
| 최대 측정거리 | 12m |
| 스캔 주파수 | 10Hz (100ms/회전) |
| 보조 센서 | PIR 센서 |

---

## 폴더 구조

```
embedded/
├── CMakeLists.txt
├── README.md
├── src/
│   ├── main.cpp
│   ├── lidar_interface.cpp     # LD19 연결 및 스캔
│   ├── preprocessor.cpp        # 좌표 변환, 노이즈 필터
│   ├── clustering.cpp          # DBSCAN 클러스터링
│   ├── event_detector.cpp      # 객체 이탈 탐지 로직
│   ├── http_sender.cpp         # FastAPI HTTP POST 전송
│   └── udp_sender.cpp          # Unity UDP 전송
├── include/
│   ├── lidar_interface.hpp
│   ├── preprocessor.hpp
│   ├── clustering.hpp
│   ├── event_detector.hpp
│   ├── http_sender.hpp
│   └── udp_sender.hpp
└── third_party/
    └── ldlidar_stl_sdk/
```

| 파일 | 역할 |
|---|---|
| `lidar_interface` | ldlidar_stl_sdk를 래핑하여 LD19 시리얼 연결, 극좌표 스캔 데이터(angle, distance, intensity) 수집 |
| `preprocessor` | 거리 기반 노이즈 필터링(30mm~12000mm) 및 극좌표 → 직교좌표(X, Y) 변환 |
| `clustering` | DBSCAN 알고리즘으로 직교좌표 포인트를 클러스터링, 중심점 및 포인트 리스트 산출 |
| `event_detector` | 프레임 간 클러스터 추적(그리디 최근접 매칭) 및 이동→정지 전이를 통한 이탈(투기) 판정 |
| `http_sender` | 이탈 이벤트 발생 시 libcurl로 FastAPI 서버에 HTTP POST, 실패 시 로컬 JSONL 파일 큐잉 + 백그라운드 재전송 |
| `udp_sender` | 매 프레임 클러스터 메타데이터 + 이벤트를 nlohmann/json으로 직렬화하여 Unity에 UDP 전송 |

---

## 데이터 흐름

```
LD19 LiDAR (serial /dev/ttyUSB0, 230400 baud)
    │
    ▼
lidar_interface: 극좌표 (angle, distance, intensity) 수집
    │
    ▼
preprocessor: 노이즈 제거 (30mm~12000mm) → 직교좌표 (X, Y) 변환
    │
    ▼
clustering: DBSCAN (ε=150mm, minPts=5) → 클러스터 중심점 및 포인트 리스트
    │
    ▼
event_detector: 이동 객체 추적 → 이탈(투기) 판정
    │                         │
    ▼                         ▼
http_sender               udp_sender
(FastAPI :8000)           (Unity :9000)
JSON POST 전송            JSON UDP 전송
```

---

## 핵심 파라미터

| 파라미터 | 값 | 설명 |
|---|---|---|
| 스캔 주기 | 100ms (10Hz) | LD19 스캔 주파수 기준 메인 루프 주기 |
| 노이즈 필터 최소거리 | 30mm | 이하 제거 — 센서 근접 노이즈 |
| 노이즈 필터 최대거리 | 12000mm | 초과 제거 — LD19 최대 측정거리 초과 무효 데이터 |
| DBSCAN epsilon | 150mm | 이웃 포인트 탐색 반경 |
| DBSCAN minPoints | 5 | 코어 포인트 판정 최소 이웃 수 |
| 정지 판정 기준 | 50mm | 연속 프레임 간 중심점 이동거리가 이 값 미만이면 정지로 판정 |
| 이탈 판정 프레임 수 | 3프레임 | 이동 이력이 있는 객체가 연속 3프레임 정지 시 이탈(투기)로 판정 |
| HTTP 엔드포인트 | `https://api.ecowarden.systems/api/dumping-event` | FastAPI 이벤트 수신 서버 |
| HTTP 타임아웃 | 3000ms | libcurl 연결 + 응답 타임아웃 |
| UDP 목적지 | `192.168.20.52:9090` | Unity 클라이언트 수신 주소 |
| UDP 최대 패킷 크기 | 65507 bytes | UDP datagram 최대 payload, 초과 시 클러스터 자동 잘림 |

---

## 의존성

| 라이브러리 | 용도 | 설치 |
|---|---|---|
| [ldlidar_stl_sdk](https://github.com/ldrobotSensorTeam/ldlidar_stl_sdk) | LD19 LiDAR 드라이버 | `third_party/`에 git clone |
| [nlohmann/json](https://github.com/nlohmann/json) v3.11.3 | JSON 직렬화 | CMake FetchContent 자동 다운로드 |
| [libcurl](https://curl.se/libcurl/) | HTTP POST 전송 | `sudo apt install libcurl4-openssl-dev` |
| POSIX socket | UDP 전송 | Linux 기본 제공 |
| [CMake](https://cmake.org/) ≥ 3.16 | 빌드 시스템 | `sudo apt install cmake` |

---

## 빌드 방법

### 1. 시스템 패키지 설치

```bash
sudo apt update
sudo apt install -y build-essential cmake git libcurl4-openssl-dev
```

### 2. LiDAR SDK 클론

```bash
cd embedded/third_party
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_sdk.git
```

### 3. CMake 빌드

```bash
cd embedded
mkdir build && cd build
cmake ..
make -j$(nproc)
```

빌드 완료 시 `build/ld19_lidar_app` 바이너리가 생성됩니다.

### 4. 시리얼 포트 권한 설정 (선택)

매번 `sudo` 없이 실행하려면:

```bash
sudo usermod -aG dialout $USER
# 로그아웃 후 다시 로그인
```

---

## 실행 방법

### 기본 실행

```bash
sudo ./build/ld19_lidar_app
```

### 옵션 지정 실행

```bash
sudo ./build/ld19_lidar_app [시리얼포트] [FastAPI_URL] [Unity_IP:PORT]
```

```bash
# 예시: 모든 옵션 지정
sudo ./build/ld19_lidar_app /dev/ttyUSB0 http://192.168.1.10:8000/api/events 192.168.20.52:9090
```

| 인자 | 기본값 | 설명 |
|---|---|---|
| `argv[1]` | `/dev/ttyUSB0` | LD19 시리얼 포트 |
| `argv[2]` | `https://api.ecowarden.systems/api/dumping-event` | FastAPI 이벤트 수신 엔드포인트 |
| `argv[3]` | `192.168.20.52:9090` | Unity UDP 수신 주소 |

### 예시 출력

```
=== LD19 LiDAR — Full Pipeline ===
Serial     : /dev/ttyUSB0 @ 230400
Filter     : 30 ~ 12000 mm
DBSCAN     : eps=150mm, minPts=5
Tracker    : stop=50mm, depart=3 frames
HTTP API   : https://api.ecowarden.systems/api/dumping-event
UDP Unity  : 192.168.20.52:9090

[INFO] LiDAR started. Press Ctrl+C to stop.

── Frame #1  (raw=456, 10.2 Hz) ──────────────────────────────
  Filtered: 412 | Clusters: 3
    C0: ( +1523,   +803) mm  d= 1722  pts=87
    C1: ( -2100,  +1450) mm  d= 2552  pts=52
    C2: (  +400,   -200) mm  d=  447  pts=31
  Tracks (3):
      ID       State    X(mm)    Y(mm)  StopN   Age
       0      MOVING    +1523     +803      0    12
       1  STATIONARY    -2100    +1450      2     8
       2      MOVING     +400     -200      0     5

── Frame #5  (raw=461, 10.1 Hz) ──────────────────────────────
  ...
  ** DEPARTURE: track=1 pos=(-2100,+1450) stop=3 frames **
  [NOTIFIER] Sent: {"timestamp":"2026-03-27T12:34:56.789Z", ...}
```

`Ctrl+C`로 종료하면 미전송 큐를 최종 flush한 뒤 정리합니다.

---

## 전송 패킷 구조

### HTTP POST — 이탈 이벤트 (FastAPI)

이탈(투기) 판정 시에만 전송됩니다.

**엔드포인트**: `POST https://api.ecowarden.systems/api/dumping-event`

```json
{
  "timestamp": "2026-01-01T00:00:00Z",
  "x": 800.0,
  "y": 200.0,
  "cluster_id": 1,
  "type": "abandoned"
}
```

| 필드 | 타입 | 설명 |
|---|---|---|
| `timestamp` | string | ISO 8601 이탈 판정 시각 |
| `x` | float | 이탈 위치 X (mm) |
| `y` | float | 이탈 위치 Y (mm) |
| `cluster_id` | int | 추적 클러스터 ID |
| `type` | string | 항상 `"abandoned"` |

전송 실패 시 `/tmp/ld19_event_queue.jsonl`에 JSONL로 큐잉되며, 백그라운드 스레드가 10초 주기로 재전송합니다.

### UDP — 클러스터 메타데이터 (Unity)

매 프레임(10Hz) 전송됩니다.

**목적지**: `192.168.20.52:9090`

```json
{
  "type": "FRAME",
  "objects": [
    {
      "id": 0,
      "x": 1200.0,
      "y": 350.0,
      "type": "normal"
    },
    {
      "id": 1,
      "x": 800.0,
      "y": 200.0,
      "type": "abandoned"
    }
  ]
}
```

| 필드 | 타입 | 설명 |
|---|---|---|
| `type` | string | 항상 `"FRAME"` |
| `objects[].id` | int | 클러스터 번호 |
| `objects[].x` | float | 클러스터 중심 X (mm) |
| `objects[].y` | float | 클러스터 중심 Y (mm) |
| `objects[].type` | string | `"normal"` 또는 `"abandoned"` |

---

## 트러블슈팅

### 1. `/dev/ttyUSB0` Permission denied

```
[ERROR] Start failed: ConnectionFailed: serial port open or comm failed
```

**원인**: 현재 사용자에게 시리얼 포트 접근 권한이 없습니다.

**해결**:
```bash
# 임시 해결
sudo ./build/ld19_lidar_app

# 영구 해결
sudo usermod -aG dialout $USER
# 로그아웃 후 재로그인, 이후 sudo 없이 실행 가능
```

LD19 USB가 다른 포트에 잡힌 경우:
```bash
ls /dev/ttyUSB*
# 출력된 포트를 인자로 지정
./build/ld19_lidar_app /dev/ttyUSB1
```

---

### 2. ldlidar_stl_sdk 빌드 실패

```
CMake Error: ldlidar_stl_sdk not found.
```

**원인**: `third_party/ldlidar_stl_sdk/` 디렉토리가 없거나 비어있습니다.

**해결**:
```bash
cd embedded/third_party
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_sdk.git
```

SDK 소스 컴파일 오류 시:
```bash
# GCC/G++ 버전 확인 (C++17 필요)
g++ --version   # 7.0 이상 필요

# Ubuntu 22.04 기본 GCC 11은 문제없음
sudo apt install build-essential
```

---

### 3. LD19 스캔 데이터 없음 (0 points)

```
[WARN] Scan timeout — retrying...
```

**원인 및 확인 순서**:

1. **물리적 연결 확인**: USB 케이블 재연결, LD19 모터 회전 여부 확인
2. **포트 확인**:
   ```bash
   ls /dev/ttyUSB*
   dmesg | tail -20   # USB 장치 인식 로그 확인
   ```
3. **baudrate 확인**: LD19는 반드시 **230400** baud로 통신해야 합니다
4. **다른 프로세스 점유**: 시리얼 포트를 다른 프로그램이 사용 중인지 확인
   ```bash
   sudo fuser /dev/ttyUSB0
   ```
5. **전원 부족**: USB 허브 대신 본체 USB 포트에 직접 연결

---

### 4. UDP 패킷 Unity에서 수신 안 됨

**확인 순서**:

1. **포트 번호 일치**: C++ 앱 인자(`argv[3]`)와 Unity 스크립트의 `listenPort`가 동일한지 확인
2. **방화벽 확인**:
   ```bash
   sudo ufw status
   sudo ufw allow 9000/udp
   ```
3. **네트워크 확인** (원격 PC인 경우):
   ```bash
   # C++ 측에서 전송 확인
   # 별도 터미널에서 수신 테스트
   nc -lu 9000
   ```
4. **IP 주소 확인**: `127.0.0.1`은 로컬 전용. 다른 PC로 보내려면 해당 PC의 실제 IP를 지정
   ```bash
   ./build/ld19_lidar_app /dev/ttyUSB0 https://api.ecowarden.systems/api/dumping-event 192.168.20.52:9090
   ```
5. **Unity 측 오류 로그**: Unity Console 창에서 `[LD19-JSON]` 로그 확인

---

### 5. HTTP POST 연결 거부 (FastAPI 미실행)

```
[NOTIFIER] curl error: Connection refused
[NOTIFIER] Queued (send failed): {"timestamp":"...", ...}
```

**원인**: FastAPI 서버가 실행되지 않았거나 엔드포인트 주소가 잘못되었습니다.

**해결**:

1. **서버 실행 확인**:
   ```bash
   cd server
   pip install fastapi uvicorn
   uvicorn main:app --host 0.0.0.0 --port 8000
   ```
2. **엔드포인트 접속 테스트**:
   ```bash
   curl https://api.ecowarden.systems/health
   # {"status":"ok"} 응답 확인
   ```
3. **원격 서버인 경우**: `argv[2]`에 정확한 URL 지정
   ```bash
   ./build/ld19_lidar_app /dev/ttyUSB0 https://api.ecowarden.systems/api/dumping-event
   ```

서버가 일시적으로 다운되어도 이벤트는 `/tmp/ld19_event_queue.jsonl`에 자동 큐잉되며, 서버 복구 후 백그라운드 스레드가 10초 주기로 재전송합니다.

---

## 라이선스

이 프로젝트는 MIT 라이선스를 따릅니다.

자세한 내용은 [LICENSE](../LICENSE) 파일을 참고하세요.

서드파티 라이브러리의 라이선스 정보는 [THIRD_PARTY_LICENSES.md](../THIRD_PARTY_LICENSES.md)를 참고하세요.
