#pragma once

#include <string>

namespace ld19 {

class CameraModule {
public:
    explicit CameraModule(int device_id = 0);
    ~CameraModule();

    // OpenCV 버퍼를 비우고 최신 프레임을 캡처한 뒤 JPEG 압축 -> Base64 문자열 반환
    // 에러 시 빈 문자열 반환
    std::string CaptureBase64();

private:
    int device_id_;
    void* cap_; // cv::VideoCapture pointer (OpenCV 헤더 은닉)
};

} // namespace ld19
