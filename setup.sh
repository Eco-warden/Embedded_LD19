#!/bin/bash
# LD19 LiDAR 프로젝트 초기 설정 스크립트
# Ubuntu 22.04 에서 실행
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SDK_DIR="${SCRIPT_DIR}/thirdparty/ldlidar_stl_sdk"

echo "=== LD19 LiDAR Project Setup ==="

# 1. 시스템 의존성 설치
echo "[1/4] Installing dependencies..."
sudo apt update
sudo apt install -y build-essential cmake git libcurl4-openssl-dev python3-pip
pip3 install fastapi uvicorn

# 2. ldlidar_stl_sdk 클론
if [ ! -d "${SDK_DIR}" ]; then
    echo "[2/4] Cloning ldlidar_stl_sdk..."
    mkdir -p "${SCRIPT_DIR}/thirdparty"
    git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_sdk.git "${SDK_DIR}"
else
    echo "[2/4] ldlidar_stl_sdk already exists, skipping clone."
fi

# 3. 빌드
echo "[3/4] Building..."
mkdir -p "${SCRIPT_DIR}/build"
cd "${SCRIPT_DIR}/build"
cmake ..
make -j"$(nproc)"

# 4. 안내
echo ""
echo "[4/4] Build complete!"
echo ""
echo "── FastAPI 서버 실행 ──────────────────────────────────"
echo "  cd ${SCRIPT_DIR}/server"
echo "  uvicorn main:app --host 0.0.0.0 --port 8000"
echo ""
echo "── LiDAR 앱 실행 ─────────────────────────────────────"
echo "  sudo ${SCRIPT_DIR}/build/ld19_lidar_app [포트] [API_URL]"
echo ""
echo "  예시:"
echo "  sudo ./build/ld19_lidar_app"
echo "  sudo ./build/ld19_lidar_app /dev/ttyUSB0 http://192.168.1.100:8000/api/v1/events"
echo ""
echo "── 시리얼 포트 권한 영구 설정 ─────────────────────────"
echo "  sudo usermod -aG dialout \$USER"
echo "  (로그아웃 후 다시 로그인)"
