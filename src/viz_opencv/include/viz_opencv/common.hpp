#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

typedef Eigen::Matrix<double,4,4> Matrix4d;
typedef Eigen::Matrix<double,4,3> Matrix4_3d;
typedef Eigen::Matrix<double,3,3> Matrix3d;
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