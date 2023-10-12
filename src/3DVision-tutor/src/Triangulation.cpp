#include <3DVision-tutor/common.hpp>

class Calculator
{
    public:
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

    private:
};

class Camera
{
    public:
        Matrix3d K_eig;
        pose_t pose_eig;

        cv::Mat image, K_mat;
        cv::Affine3f pose_aff;
        double base_line = 0.54;

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
                if(x > this->image.cols || y > this->image.rows)
                    continue;
                    
                this->image.at<cv::Vec3b>(y, x)[0] = 0;
                this->image.at<cv::Vec3b>(y, x)[1] = 255;
                this->image.at<cv::Vec3b>(y, x)[2] = 0;
            }
        };

    private:
};

int main(int argc, char** argv)
{
    /*------------------------------point cloud 및 stereo camera pose 초기화------------------------------*/
    Camera left_cam, right_cam;
    right_cam.pose_eig.t << -right_cam.base_line, 0.0, 0.0;
    left_cam.pose_aff = Calculator::Eig_to_Aff(left_cam.pose_eig);
    right_cam.pose_aff = Calculator::Eig_to_Aff(right_cam.pose_eig);

    int num_points = 5;
    cv::Mat point_cloud(1, num_points*num_points, CV_32FC3);
    float gap = 0.1;
    float x = -gap*2.0, y = -gap*2.0, z = 2.0;
    
    // cv::Point3f* data = point_cloud.ptr<cv::Point
    
    for(int i = 0; i < point_cloud.cols; i++)
    {
        point_cloud.at<cv::Point3f>(i).x = x;
        point_cloud.at<cv::Point3f>(i).y = y;
        point_cloud.at<cv::Point3f>(i).z = z;
        x += gap;
        // y += 0.5;
        // z += 0.5;
        if(i % num_points == num_points-1)
        {
            x = -gap*2.0;
            y += gap;
        }
        // std::cout << point_cloud.at<cv::Point3f>(i) << std::endl;
    }

    /*------------------------------point cloud 기반으로 이미지 만들기------------------------------*/

    left_cam.SetImage(point_cloud);
    right_cam.SetImage(point_cloud);
    cv::imshow("left_cam.image",left_cam.image);
    cv::imshow("right_cam.image",right_cam.image);
    cv::waitKey(0);
    cv::destroyAllWindows();

    /*------------------------------3D viewer------------------------------*/
    
    cv::viz::Viz3d myWindow("Coordinate Frame");
    myWindow.showWidget("Left_Cam", cv::viz::WCoordinateSystem());
    myWindow.showWidget("Right_Cam", cv::viz::WCoordinateSystem(), right_cam.pose_aff);
    myWindow.setWindowSize(cv::Size(1280,960));

    cv::viz::WCloud cloud_widget(point_cloud, cv::viz::Color::green());
    myWindow.showWidget("point_cloud", cloud_widget);

    while(!myWindow.wasStopped())
    {
        myWindow.spinOnce(1, true);
    }
    
    return 0;
}