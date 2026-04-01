#include "camera_module.h"
#include <iostream>

#ifdef USE_OPENCV
#include <opencv2/opencv.hpp>
#include <vector>

static const std::string base64_chars = 
             "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
             "abcdefghijklmnopqrstuvwxyz"
             "0123456789+/";

static std::string Base64Encode(unsigned char const* bytes_to_encode, unsigned int in_len) {
    std::string ret;
    int i = 0, j = 0;
    unsigned char char_array_3[3], char_array_4[4];

    while (in_len--) {
        char_array_3[i++] = *(bytes_to_encode++);
        if (i == 3) {
            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
            char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
            char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
            char_array_4[3] = char_array_3[2] & 0x3f;

            for(i = 0; (i <4) ; i++)
                ret += base64_chars[char_array_4[i]];
            i = 0;
        }
    }

    if (i) {
        for(j = i; j < 3; j++)
            char_array_3[j] = '\0';

        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;

        for (j = 0; (j < i + 1); j++)
            ret += base64_chars[char_array_4[j]];

        while((i++ < 3))
            ret += '=';
    }
    return ret;
}
#endif

namespace ld19 {

CameraModule::CameraModule(int device_id) : device_id_(device_id), cap_(nullptr) {}

CameraModule::~CameraModule() {
#ifdef USE_OPENCV
    if (cap_) {
        delete static_cast<cv::VideoCapture*>(cap_);
        cap_ = nullptr;
    }
#endif
}

std::string CameraModule::CaptureBase64() {
#ifdef USE_OPENCV
    if (!cap_) {
        auto* vcap = new cv::VideoCapture(device_id_, cv::CAP_V4L2);
        vcap->set(cv::CAP_PROP_FRAME_WIDTH, 1920);
        vcap->set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
        // 야간 환경 및 USB 대역폭 최적화를 위해 MJPG 설정 고려 가능
        // vcap->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        cap_ = vcap;
    }

    cv::VideoCapture* vcap = static_cast<cv::VideoCapture*>(cap_);
    if (!vcap->isOpened()) {
        std::cerr << "[CAMERA] Failed to open device " << device_id_ << "\n";
        return "";
    }

    cv::Mat frame;
    // 이전 버퍼에 쌓여있던 과거 낡은 프레임을 날려버리고 최신 프레임을 얻기 위한 dummy read
    // 카메라 자동 노출(Auto Exposure) 적응 시간 확보
    vcap->read(frame);
    vcap->read(frame);
    vcap->read(frame);
    vcap->read(frame);

    if (frame.empty()) {
        std::cerr << "[CAMERA] Empty frame captured.\n";
        return "";
    }

    std::vector<uchar> buf;
    // JPEG 품질 80 설정 (용량 및 전송 속도 최적화)
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};
    if (!cv::imencode(".jpg", frame, buf, params)) {
        std::cerr << "[CAMERA] JPEG Encode failed.\n";
        return "";
    }

    return Base64Encode(buf.data(), buf.size());
#else
    std::cerr << "[CAMERA] OpenCV is not installed. Returning empty string.\n";
    return "";
#endif
}

} // namespace ld19
