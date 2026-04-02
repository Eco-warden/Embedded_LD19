# Copyright (c) 2026 김승연
# MIT License - See LICENSE file for details

cmake_minimum_required(VERSION 3.16)
project(embedded_lidar_module
  VERSION   1.0.0
  LANGUAGES CXX
)

add_compile_options(-D__SANE_USERSPACE_TYPES__)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── ldlidar_stl_sdk (git clone into thirdparty/) ──────────────────────
set(LDLIDAR_SDK_DIR ${CMAKE_SOURCE_DIR}/thirdparty/ldlidar_stl_sdk)

if(NOT EXISTS ${LDLIDAR_SDK_DIR}/CMakeLists.txt)
  message(FATAL_ERROR
    "ldlidar_stl_sdk not found.\n"
    "Run:  git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_sdk.git "
    "${LDLIDAR_SDK_DIR}"
  )
endif()

# SDK 소스 수집
file(GLOB_RECURSE LDLIDAR_SDK_SOURCES
  ${LDLIDAR_SDK_DIR}/ldlidar_driver/src/**/*.cpp
)

# SDK 헤더 디렉토리
set(LDLIDAR_SDK_INCLUDE_DIRS
  ${LDLIDAR_SDK_DIR}/ldlidar_driver/include
  ${LDLIDAR_SDK_DIR}/ldlidar_driver/include/core
  ${LDLIDAR_SDK_DIR}/ldlidar_driver/include/dataprocess
  ${LDLIDAR_SDK_DIR}/ldlidar_driver/include/filter
  ${LDLIDAR_SDK_DIR}/ldlidar_driver/include/logger
  ${LDLIDAR_SDK_DIR}/ldlidar_driver/include/networkcom
  ${LDLIDAR_SDK_DIR}/ldlidar_driver/include/serialcom
)

# SDK를 정적 라이브러리로 빌드
add_library(ldlidar_driver STATIC ${LDLIDAR_SDK_SOURCES})
target_include_directories(ldlidar_driver PUBLIC ${LDLIDAR_SDK_INCLUDE_DIRS})
target_link_libraries(ldlidar_driver PUBLIC pthread)
target_compile_definitions(ldlidar_driver PUBLIC __SANE_USERSPACE_TYPES__)

# ── libcurl ───────────────────────────────────────────────────────────
find_package(CURL REQUIRED)

# ── OpenCV (Optional, for Camera Module) ──────────────────────────────
find_package(OpenCV QUIET)
if(OpenCV_FOUND)
  add_definitions(-DUSE_OPENCV)
  message(STATUS "OpenCV found, CameraModule will be compiled with camera support.")
else()
  message(WARNING "OpenCV not found, CameraModule will return empty images.")
endif()

# ── nlohmann/json (header-only, FetchContent) ────────────────────────
include(FetchContent)
FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG        v3.11.3
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(nlohmann_json)

# ── Application ───────────────────────────────────────────────────────
add_executable(ld19_lidar_app
  src/ld19_lidar.cpp
  src/scan_processor.cpp
  src/background_filter.cpp
  src/cluster_tracker.cpp
  src/event_notifier.cpp
  src/udp_sender.cpp
  src/json_packet.cpp
  src/pir_sensor.cpp
  src/camera_module.cpp
  src/main.cpp
)

target_include_directories(ld19_lidar_app PRIVATE
  ${CMAKE_SOURCE_DIR}/include
  ${LDLIDAR_SDK_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIRS}
)

target_link_libraries(ld19_lidar_app PRIVATE
  ldlidar_driver
  CURL::libcurl
  nlohmann_json::nlohmann_json
)

target_compile_definitions(ld19_lidar_app PRIVATE __SANE_USERSPACE_TYPES__)

if(OpenCV_FOUND)
  target_include_directories(ld19_lidar_app PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(ld19_lidar_app PRIVATE ${OpenCV_LIBS})
endif()

# ── Simulation (센서 없이 파이프라인 테스트) ───────────────────────────
add_executable(ld19_sim
  src/scan_processor.cpp
  src/background_filter.cpp
  src/cluster_tracker.cpp
  src/event_notifier.cpp
  src/udp_sender.cpp
  src/json_packet.cpp
  src/ld19_lidar.cpp
  src/pir_sensor.cpp
  src/sim_main.cpp
)

target_include_directories(ld19_sim PRIVATE
  ${CMAKE_SOURCE_DIR}/include
  ${LDLIDAR_SDK_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIRS}
)

target_link_libraries(ld19_sim PRIVATE
  ldlidar_driver
  CURL::libcurl
  nlohmann_json::nlohmann_json
)

target_compile_definitions(ld19_sim PRIVATE __SANE_USERSPACE_TYPES__)
