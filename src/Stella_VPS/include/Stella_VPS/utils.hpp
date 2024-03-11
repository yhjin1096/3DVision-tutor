#ifndef STELLA_VPS_UTIL_UTILS_H
#define STELLA_VPS_UTIL_UTILS_H

#include <iostream>
#include <boost/filesystem.hpp>
#include <assert.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

const std::string imageExtensions[] = { ".jpg", ".jpeg", ".png", ".gif", ".bmp" };

inline void CountImages(int& num_images, const std::string& path)
{
    try {
        // 지정된 폴더 내의 모든 파일에 대해 반복
        for (const auto& entry : boost::filesystem::directory_iterator(path)) {
            // 디렉토리인 경우 건너뛰기
            if (boost::filesystem::is_directory(entry.path()))
                continue;

            // 이미지 파일인 경우 개수 증가
            for (const std::string& ext : imageExtensions) {
                if (entry.path().extension() == ext)
                    num_images++;
            }
        }

        std::cout << "Number of image files in the folder: " << num_images << std::endl;
    }
    catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
    }
}

namespace util {

static constexpr float _PI = 3.14159265358979f;
static constexpr float _PI_2 = _PI / 2.0f;
static constexpr float _TWO_PI = 2.0f * _PI;
static constexpr float _INV_TWO_PI = 1.0f / _TWO_PI;
static constexpr float _THREE_PI_2 = 3.0f * _PI_2;

inline float _cos(float v) {
    constexpr float c1 = 0.99940307f;
    constexpr float c2 = -0.49558072f;
    constexpr float c3 = 0.03679168f;

    const float v2 = v * v;
    return c1 + v2 * (c2 + c3 * v2);
}

inline float cos(float v) {
    v = v - cvFloor(v * _INV_TWO_PI) * _TWO_PI;
    v = (0.0f < v) ? v : -v;

    if (v < _PI_2) {
        return _cos(v);
    }
    else if (v < _PI) {
        return -_cos(_PI - v);
    }
    else if (v < _THREE_PI_2) {
        return -_cos(v - _PI);
    }
    else {
        return _cos(_TWO_PI - v);
    }
}

inline float sin(float v) {
    return util::cos(_PI_2 - v);
}

} // namespace util

#endif