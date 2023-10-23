/*------------------------------ ------------------------------*/
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

typedef Eigen::Matrix<double,4,4> Matrix4d;
typedef Eigen::Matrix<double,4,3> Matrix4_3d;
typedef Eigen::Matrix<double,3,3> Matrix3d;
typedef Eigen::Matrix<float,3,3> Matrix3f;
typedef Eigen::Matrix<double,2,2> Matrix2d;
typedef Eigen::Matrix<float,2,2> Matrix2f;
typedef Eigen::Matrix<double,2,1> Vector2d;
typedef Eigen::Matrix<double,3,1> Vector3d;
typedef Eigen::Matrix<double,4,1> Vector4d;
typedef Eigen::Matrix<double,5,1> Vector5d;
typedef Eigen::Matrix<double,6,1> Vector6d;
typedef Eigen::Matrix<float,2,1> Vector2f;
typedef Eigen::Matrix<float,3,1> Vector3f;
typedef Eigen::Matrix<float,6,1> Vector6f;
typedef Eigen::Matrix<float,72,1> Vector72f;
typedef Eigen::Matrix<double,36,1> Vector36d;

typedef Eigen::Matrix<double,8,1> Vector8d;
typedef Eigen::Matrix<double,12,1> Vector12d;

typedef struct _pose_t {
  _pose_t() : R( Matrix3d::Identity() ), t( Vector3d::Zero() ) {}
  _pose_t( const _pose_t & T ) : R( T.R ), t( T.t ) {}
  _pose_t( Matrix3d R, Vector3d t ) : R(R), t(t) {}

  Matrix3d R;
  Vector3d t;
} pose_t;

class Camera
{
    public:
        Matrix3f K_eig;
        pose_t pose_eig;

        cv::Mat image, K_mat;
        cv::Affine3f pose_aff; //c2w
        double base_line = 0.54;

        std::vector<cv::Mat> points_pixel_h;
        std::vector<cv::Point2f> points_pixel;

        Camera()
        {
            //parameter of kitti dataset sequence 06
            this->K_eig << 7.070912000000e+02, 0.000000000000e+00, 6.018873000000e+02,
                           0.000000000000e+00, 7.070912000000e+02, 1.831104000000e+02,
                           0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00;
            this->K_mat = (cv::Mat_<float>(3,3) << 7.070912000000e+02, 0.000000000000e+00, 6.018873000000e+02,
                                                   0.000000000000e+00, 7.070912000000e+02, 1.831104000000e+02,
                                                   0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00);
            this->image = cv::Mat::zeros(cv::Size(1226,370), CV_8UC3);
        };
        ~Camera()
        {

        };

        void SetImage(const cv::Mat& point_cloud)
        {
            for(int i = 0; i < point_cloud.cols; i++)
            {
                 cv::Vec3f tmp = this->pose_aff * point_cloud.at<cv::Vec3f>(i);
                 tmp = tmp/tmp(2);

                 cv::Mat tmp_mat(3,1,CV_32FC1);
                 tmp_mat.at<float>(0) = tmp(0);
                 tmp_mat.at<float>(1) = tmp(1);
                 tmp_mat.at<float>(2) = tmp(2);

                cv::Mat result = K_mat * tmp_mat;

                int x = std::round(result.at<float>(0)), y = std::round(result.at<float>(1));
                if(x >= this->image.cols || y >= this->image.rows ||
                   x < 0 || y < 0)
                    continue;
                    
                this->points_pixel_h.push_back(result);
                this->points_pixel.push_back(cv::Point2f(result.at<float>(0),result.at<float>(1)));
                    
                this->image.at<cv::Vec3b>(y, x)[0] = 0;
                this->image.at<cv::Vec3b>(y, x)[1] = 255;
                this->image.at<cv::Vec3b>(y, x)[2] = 0;
            }
        };

    private:
};

class Calculator
{
    public:
        static Matrix3d rpy_to_REigen(double x, double y, double z)
        {
            Matrix3d Rx, Ry, Rz, result;
            
            x = x / 180.0 * CV_PI;
            y = y / 180.0 * CV_PI;
            z = z / 180.0 * CV_PI;

            // Rx << 1.0, 0.0, 0.0,
            //       0.0, cos(x), -sin(x),
            //       0.0, sin(x), cos(x);
            
            // Ry << cos(y), 0.0, sin(y),
            //       0.0, 1.0, 0.0,
            //       -sin(y), 0.0, cos(y);

            // Rz << cos(z), -sin(z), 0.0,
            //       sin(z), cos(z), 0.0,
            //       0.0, 0.0, 1.0;
                
            // result = Rz * Ry * Rx;

            
            result = Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX())
                    * Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ());
            
            return result;
        };
        static cv::Mat rpy_to_RMat(double x, double y, double z)
        {
            Matrix3d Rx, Ry, Rz, result;
            
            x = x / 180.0 * CV_PI;
            y = y / 180.0 * CV_PI;
            z = z / 180.0 * CV_PI;

            // Rx << 1.0, 0.0, 0.0,
            //       0.0, cos(x), -sin(x),
            //       0.0, sin(x), cos(x);
            
            // Ry << cos(y), 0.0, sin(y),
            //       0.0, 1.0, 0.0,
            //       -sin(y), 0.0, cos(y);

            // Rz << cos(z), -sin(z), 0.0,
            //       sin(z), cos(z), 0.0,
            //       0.0, 0.0, 1.0;
                
            // result = Rz * Ry * Rx;

            
            result = Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX())
                    * Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ());
            
            cv::Mat result_mat;
            result_mat = (cv::Mat_<float>(3,3) << result(0,0), result(0,1), result(0,2),
                                                result(1,0), result(1,1), result(1,2),
                                                result(2,0), result(2,1), result(2,2));
            
            return result_mat;
        };
        static cv::Affine3f Eig_to_Aff(const pose_t& pose)
        {
            cv::Affine3f result;
            cv::Mat rot;
            cv::Vec3f tr;
            
            rot = (cv::Mat_<float>(3,3) << pose.R(0,0), pose.R(0,1), pose.R(0,2),
                                           pose.R(1,0), pose.R(1,1), pose.R(1,2),
                                           pose.R(2,0), pose.R(2,1), pose.R(2,2));
            tr = cv::Vec3f(pose.t(0), pose.t(1), pose.t(2));
            
            result.rotation(rot);
            result.translation(tr);

            return result;
        };
        
        static cv::Point2f pixel2cam(const cv::Point2f &p, const cv::Mat &K)
        {
            return cv::Point2f(
                (p.x - K.at<float>(0, 2)) / K.at<float>(0, 0),
                (p.y - K.at<float>(1, 2)) / K.at<float>(1, 1));
        }

        //https://github.com/luigifreda/slamplay
        static void Triangulation(const Camera& left_cam, const Camera& right_cam)
        {
            cv::Mat T1 = (cv::Mat_<float>(3, 4) << left_cam.pose_eig.R(0,0), left_cam.pose_eig.R(0,1), left_cam.pose_eig.R(0,2), left_cam.pose_eig.t(0),
                                                   left_cam.pose_eig.R(1,0), left_cam.pose_eig.R(1,1), left_cam.pose_eig.R(1,2), left_cam.pose_eig.t(1),
                                                   left_cam.pose_eig.R(2,0), left_cam.pose_eig.R(2,1), left_cam.pose_eig.R(2,2), left_cam.pose_eig.t(2));
            cv::Mat T2 = (cv::Mat_<float>(3, 4) << right_cam.pose_eig.R(0,0), right_cam.pose_eig.R(0,1), right_cam.pose_eig.R(0,2), right_cam.pose_eig.t(0),
                                                   right_cam.pose_eig.R(1,0), right_cam.pose_eig.R(1,1), right_cam.pose_eig.R(1,2), right_cam.pose_eig.t(1),
                                                   right_cam.pose_eig.R(2,0), right_cam.pose_eig.R(2,1), right_cam.pose_eig.R(2,2), right_cam.pose_eig.t(2));
            std::vector<cv::Point2f> pts_1, pts_2;
    
            for(int i = 0; i < left_cam.points_pixel_h.size(); i++)
            {
                pts_1.push_back(pixel2cam(cv::Point2f(left_cam.points_pixel_h[i].at<float>(0), left_cam.points_pixel_h[i].at<float>(1)), left_cam.K_mat));
                pts_2.push_back(pixel2cam(cv::Point2f(right_cam.points_pixel_h[i].at<float>(0), right_cam.points_pixel_h[i].at<float>(1)), right_cam.K_mat));
            }
            
            cv::Mat pts_4d;
            cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);
            for (int i = 0; i < pts_4d.cols; i++)
            {
                cv::Mat x = pts_4d.col(i);

                if(x.at<float>(3, 0) != 0)
                    x /= x.at<float>(3, 0); // Normalized

                cv::Point3d p(
                    x.at<float>(0, 0),
                    x.at<float>(1, 0),
                    x.at<float>(2, 0));
                // points.push_back(p);
                std::cout << p << std::endl;
            }
        }
        
        //https://www.cv-learn.com/20230616-slam-book-kr/
        static void Triangulation_v2(const Camera& left_cam, const Camera& right_cam)
        {
            for(int i = 0; i < left_cam.points_pixel_h.size(); i++)
            {
                Vector3f left_sp, right_sp;
                left_sp << left_cam.points_pixel_h[i].at<float>(0),
                           left_cam.points_pixel_h[i].at<float>(1),
                           left_cam.points_pixel_h[i].at<float>(2);
                right_sp << right_cam.points_pixel_h[i].at<float>(0),
                            right_cam.points_pixel_h[i].at<float>(1),
                            right_cam.points_pixel_h[i].at<float>(2);

                left_sp = left_cam.K_eig.inverse() * left_sp;
                right_sp = right_cam.K_eig.inverse() * right_sp;

                //non-linear triangulation
                Matrix2f A;
                Vector3f base_line_;
                Vector2f b, result;

                base_line_ << -left_cam.base_line, 0., 0.;

                b << -base_line_.transpose()*left_sp, -base_line_.transpose()*right_sp;

                A << left_sp.transpose()*left_sp, -right_sp.transpose()*left_sp,
                    left_sp.transpose()*right_sp, -right_sp.transpose()*right_sp;
                
                result = A.inverse()*b;
                
                Vector3f F, G;
                F = result(0) * left_sp;
                G = base_line_ + result(1) * right_sp;

                std::cout << ((F+G)/2.0 + Vector3f(left_cam.base_line,0,0)).transpose() << std::endl;
            }


        }
    
        static void PoseEstimation(const Camera& first_cam, const Camera& second_cam)
        {
          
        }

    private:
};