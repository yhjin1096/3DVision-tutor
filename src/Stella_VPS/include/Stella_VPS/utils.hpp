#ifndef STELLA_VPS_UTIL_UTILS_H
#define STELLA_VPS_UTIL_UTILS_H

#include <iostream>
#include <boost/filesystem.hpp>
#include <assert.h>
#include <ctype.h>
#include <algorithm>
#include <iterator>
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"


#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

typedef Eigen::Matrix<double,4,4> Matrix4d;
typedef Eigen::Matrix<double,4,3> Matrix4_3d;
typedef Eigen::Matrix<double,3,3> Matrix3d;
typedef Eigen::Matrix<double,3,4> Matrix3_4d;
typedef Eigen::Matrix<double,2,2> Matrix2d;
typedef Eigen::Matrix<double,2,1> Vector2d;
typedef Eigen::Matrix<double,3,1> Vector3d;
typedef Eigen::Matrix<double,4,1> Vector4d;
typedef Eigen::Matrix<double,5,1> Vector5d;
typedef Eigen::Matrix<double,6,1> Vector6d;
typedef Eigen::Matrix<float,2,1> Vector2f;
typedef Eigen::Matrix<float,6,1> Vector6f;
typedef Eigen::Matrix<float,72,1> Vector72f;
typedef Eigen::Matrix<double,36,1> Vector36d;
typedef Eigen::Matrix<double,8,1> Vector8d;
typedef Eigen::Matrix<double,12,1> Vector12d;

class FrameMono
{
    public:
        cv::Mat image;
        cv::Mat R, t;
        double focal = 718.8560;
        cv::Point2d pp = cv::Point2d(607.1928, 185.2157);
    private:
};

class GTPose
{
    public:
        std::vector<cv::Mat> rotations;
        std::vector<cv::Mat> translations;

        void readGTPose(const std::string& path)
        {
            std::ifstream file(path);
            std::string line, word;

            if(file.is_open())
            {
                while(getline(file, line))
                {
                    int i = 0, j = 0;
                    Matrix3_4d pose;
                    std::stringstream ss(line);
                    while(getline(ss, word, ' '))
                    {
                        pose(j,i) = std::stod(word);
                        i++;
                        if(i==4)
                        {
                            i=0;
                            j++;
                        }
                    }
                    cv::Mat R = (cv::Mat_<double>(3,3) << pose.topLeftCorner(3,3)(0,0), pose.topLeftCorner(3,3)(0,1), pose.topLeftCorner(3,3)(0,2),
                                                          pose.topLeftCorner(3,3)(1,0), pose.topLeftCorner(3,3)(1,1), pose.topLeftCorner(3,3)(1,2),
                                                          pose.topLeftCorner(3,3)(2,0), pose.topLeftCorner(3,3)(2,1), pose.topLeftCorner(3,3)(2,2));
                    cv::Mat t(3, 1, CV_64FC1);
                    t.at<double>(0,0) = pose.topRightCorner(3,1)(0,0);
                    t.at<double>(1,0) = pose.topRightCorner(3,1)(1,0);
                    t.at<double>(2,0) = pose.topRightCorner(3,1)(2,0);
                    
                    rotations.push_back(R);
                    translations.push_back(t);
                }
                file.close();
            }
            else
            {
                std::cout << "file not found" << std::endl;
                exit(0);
            }
        };
    private:
};

inline double calculateRotationError(const cv::Mat& R_gt, const cv::Mat& R_est) {
    // Calculate the difference rotation matrix
    cv::Mat R_diff = R_gt.t() * R_est;

    // Trace of R_diff
    double trace = R_diff.at<double>(0, 0) + R_diff.at<double>(1, 1) + R_diff.at<double>(2, 2);

    // Calculate the rotation error in radians
    double theta = acos(std::max(std::min((trace - 1) / 2.0, 1.0), -1.0));

    // Convert radians to degrees
    double theta_deg = theta * (180.0 / CV_PI);

    return theta_deg;
}

inline double calculateTranslationError(const cv::Mat& t_gt, const cv::Mat& t_est)
{
    double error;

    cv::Mat t_diff = t_gt - t_est;
    error = cv::norm(t_diff);

    return error;
}

const std::string imageExtensions[] = { ".jpg", ".jpeg", ".png", ".gif", ".bmp" };

inline void CountImages(int& num_images, const std::string& path)
{
    num_images = 0;
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