#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <opencv2/viz.hpp>
#include <opencv2/viz/widgets.hpp>

#include <iostream>
#include <fstream>
#include <ctype.h>
#include <algorithm>
#include <iterator>
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>
#include <boost/filesystem.hpp>
#include <thread>

#include "ceres/ceres.h"
#include "ceres/rotation.h"

class Camera
{
    public:
        double focal = 718.8560;
        cv::Point2d pp = cv::Point2d(607.1928, 185.2157);
        cv::Mat image, gray_image;
        cv::Mat intrinsic_Mat, projection_Mat;
        cv::Mat rot_rodrigues = cv::Mat::zeros(3, 1, CV_64F), translation = cv::Mat::zeros(3, 1, CV_64F);
        std::vector<double> ceres_cam_pose = std::vector<double>(6, 0.0); // cam 기준 world
        cv::Mat world_to_cam_pose = cv::Mat::eye(4, 4, CV_64F); //cam 기준 world
        cv::Mat cam_to_world_pose = cv::Mat::eye(4, 4, CV_64F); //world 기준 cam

        std::vector<cv::Point2f> keypoints;
        std::vector<cv::Point2d> keypoints_d;

        void loadImage(std::string path)
        {
            image = cv::imread(path);
            cv::cvtColor(image, gray_image, CV_8UC1);
        }
};

class Node
{
    public:
        Camera left_cam, right_cam;
        cv::Mat points3D;
        std::vector<double> landmarks;
        double base_line = 0.5371657188644179;
        
        Node()
        {
            left_cam.projection_Mat = (cv::Mat_<double>(3,4) << 7.188560000000e+02, 0.000000000000e+00, 6.071928000000e+02, 0.000000000000e+00,
                                                               0.000000000000e+00, 7.188560000000e+02, 1.852157000000e+02, 0.000000000000e+00,
                                                               0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 0.000000000000e+00);
            left_cam.intrinsic_Mat = left_cam.projection_Mat(cv::Rect(0,0,3,3)).clone();

            right_cam.projection_Mat = (cv::Mat_<double>(3,4) << 7.188560000000e+02, 0.000000000000e+00, 6.071928000000e+02, -3.861448000000e+02,
                                                                0.000000000000e+00, 7.188560000000e+02, 1.852157000000e+02, 0.000000000000e+00,
                                                                0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 0.000000000000e+00);
            right_cam.intrinsic_Mat = left_cam.projection_Mat(cv::Rect(0,0,3,3)).clone();

            right_cam.cam_to_world_pose.at<double>(0,3) = base_line;
            right_cam.world_to_cam_pose = right_cam.cam_to_world_pose.inv();
            right_cam.ceres_cam_pose[3] = -base_line;


            // // base line 구하기
            // cv::Mat test1, test2;
            // cv::Mat addup = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);
            // cv::vconcat(left_cam.projection_Mat, addup, test1);
            // cv::vconcat(right_cam.projection_Mat, addup, test2);
            // std::cout << left_cam.intrinsic_Mat.inv() * left_cam.projection_Mat << std::endl;
            // std::cout << left_cam.intrinsic_Mat.inv() * right_cam.projection_Mat << std::endl;
        };
        void converDouble()
        {
            for(int i = 0; i < left_cam.keypoints.size(); i++)
            {
                left_cam.keypoints_d.push_back(left_cam.keypoints[i]);
                right_cam.keypoints_d.push_back(right_cam.keypoints[i]);
            }
        }
        void updatePose()
        {
            left_cam.rot_rodrigues.at<double>(0) = left_cam.ceres_cam_pose[0];
            left_cam.rot_rodrigues.at<double>(1) = left_cam.ceres_cam_pose[1];
            left_cam.rot_rodrigues.at<double>(2) = left_cam.ceres_cam_pose[2];
            left_cam.translation.at<double>(0) = left_cam.ceres_cam_pose[0];
            left_cam.translation.at<double>(1) = left_cam.ceres_cam_pose[1];
            left_cam.translation.at<double>(2) = left_cam.ceres_cam_pose[2];

            cv::Mat rotation, rigid_body_transformation;
            cv::Mat addup = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);
            cv::Rodrigues(left_cam.rot_rodrigues, rotation);

            cv::hconcat(rotation, left_cam.translation, rigid_body_transformation);
            cv::vconcat(rigid_body_transformation, addup, rigid_body_transformation);

            left_cam.world_to_cam_pose = rigid_body_transformation;
            left_cam.cam_to_world_pose = left_cam.world_to_cam_pose.inv();
        };
};

class Tracker
{
    public:
        void extractKeypoints(Camera& cam)
        {
            featureDetectionFAST(cam.gray_image, cam.keypoints);
        };
        void trackKeypoints(Node& refer, Node& query)
        {
            matchAndTrack(refer.left_cam.gray_image, refer.right_cam.gray_image,
                          query.left_cam.gray_image, query.right_cam.gray_image,
                          refer.left_cam.keypoints, refer.right_cam.keypoints,
                          query.left_cam.keypoints, query.right_cam.keypoints);
            
            // for(int i = 0; i < refer.left_cam.keypoints.size(); i++)
            // {
            //     cv::Mat refer_left_image = refer.left_cam.image.clone(), refer_right_image = refer.right_cam.image.clone();
            //     cv::Mat query_left_image = query.left_cam.image.clone(), query_right_image = query.right_cam.image.clone();

            //     cv::circle(refer_left_image, refer.left_cam.keypoints[i], 5, cv::Scalar(0,0,255), 5);
            //     cv::circle(refer_right_image, refer.right_cam.keypoints[i], 5, cv::Scalar(0,0,255), 5);
            //     cv::circle(query_left_image, query.left_cam.keypoints[i], 5, cv::Scalar(0,0,255), 5);
            //     cv::circle(query_right_image, query.right_cam.keypoints[i], 5, cv::Scalar(0,0,255), 5);
            //     cv::Mat refer_, query_, total;
            //     cv::hconcat(refer_left_image, refer_right_image, refer_);
            //     cv::hconcat(query_left_image, query_right_image, query_);
            //     cv::vconcat(query_, refer_, total);
            //     cv::imshow("total", total);
            //     char k = cv::waitKey(0);
            //     if(k==27)
            //         exit(0);
            // }
        }
        void calc3DPoints(Node& node)
        {
            cv::Mat points4D;
            cv::triangulatePoints(node.left_cam.projection_Mat,  node.right_cam.projection_Mat,
                                  node.left_cam.keypoints, node.right_cam.keypoints, points4D);
            cv::convertPointsFromHomogeneous(points4D.t(), node.points3D);
            cv::Point3f tmp;
            for(int i = 0; i < node.points3D.rows; i++)
            {
                node.landmarks.push_back(node.points3D.at<cv::Vec3f>(i,0)(0));
                node.landmarks.push_back(node.points3D.at<cv::Vec3f>(i,0)(1));
                node.landmarks.push_back(node.points3D.at<cv::Vec3f>(i,0)(2));
                // std::cout << node.points3D.at<cv::Vec3f>(i,0) << std::endl;
                // std::cout << node.landmarks[i*3+0] << ","
                //           << node.landmarks[i*3+1] << ","
                //           << node.landmarks[i*3+2] << std::endl;
            }
        };
        void calcPose(Node& refer, Node& query)
        {
            cv::Mat rotation = cv::Mat::eye(3, 3, CV_64F);
            cv::Mat translation = cv::Mat::zeros(3, 1, CV_64F);
            cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);   
            cv::Mat rvec       = cv::Mat::zeros(3, 1, CV_64F);
            cv::Mat inliers;

            int iterationsCount = 500;        // number of Ransac iterations.
            float reprojectionError = 0.1;    // maximum allowed distance to consider it an inlier.
            float confidence = 0.999;         // RANSAC successful confidence.
            bool useExtrinsicGuess = true;
            int flags =cv::SOLVEPNP_ITERATIVE;

            // rotation & translation => world to cam pose
            cv::solvePnPRansac(refer.points3D, query.left_cam.keypoints, refer.left_cam.intrinsic_Mat, distCoeffs, rvec, translation,
                               useExtrinsicGuess, iterationsCount, reprojectionError, confidence,
                               inliers, flags );
            cv::Rodrigues(rvec, rotation);

            //inlier만 남기고 제거? -> keypoints, points3D
            
            cv::Vec3f rotation_euler = rotationMatrixToEulerAngles(rotation);
            cv::Mat rigid_body_transformation; //relative pose
            cv::Mat frame_pose = refer.left_cam.cam_to_world_pose.clone();

            if(abs(rotation_euler[1])<0.1 && abs(rotation_euler[0])<0.1 && abs(rotation_euler[2])<0.1)
            {
                integrateOdometry(0 , rigid_body_transformation, frame_pose, rotation, translation);

                query.left_cam.cam_to_world_pose = frame_pose.clone();
                query.left_cam.world_to_cam_pose = query.left_cam.cam_to_world_pose.inv();
                query.left_cam.rot_rodrigues = rvec;
                query.left_cam.translation = translation;
                
                query.left_cam.ceres_cam_pose[0] = query.left_cam.rot_rodrigues.at<double>(0);
                query.left_cam.ceres_cam_pose[1] = query.left_cam.rot_rodrigues.at<double>(1);
                query.left_cam.ceres_cam_pose[2] = query.left_cam.rot_rodrigues.at<double>(2);
                query.left_cam.ceres_cam_pose[3] = query.left_cam.translation.at<double>(0);
                query.left_cam.ceres_cam_pose[4] = query.left_cam.translation.at<double>(1);
                query.left_cam.ceres_cam_pose[5] = query.left_cam.translation.at<double>(2);
                // cv::hconcat(query.left_cam.rot_rodrigues.t(), query.left_cam.translation.t(), query.left_cam.ceres_cam_pose);

            } else {

                std::cout << "Too large rotation"  << std::endl;
            }
        }

        bool isRotationMatrix(cv::Mat &R)
        {
            cv::Mat Rt;
            transpose(R, Rt);
            cv::Mat shouldBeIdentity = Rt * R;
            cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());
            
            return  norm(I, shouldBeIdentity) < 1e-6;
            
        }

    private:
        void featureDetectionFAST(const cv::Mat& img, std::vector<cv::Point2f>& points)
        {
            /* Detect features and return the image point for a single image using OpenCV FAST implementation*/
            std::vector<cv::KeyPoint> keypoints;
            int fast_threshold = 40;
            bool nonmaxSuppression = true;
            cv::FAST(img, keypoints, fast_threshold, nonmaxSuppression);
            cv::KeyPoint::convert(keypoints, points, std::vector<int>());
        }
        void deleteUntrackedFeatures(   std::vector<cv::Point2f>& points0, std::vector<cv::Point2f>& points1,
                                std::vector<cv::Point2f>& points2, std::vector<cv::Point2f>& points3,
                                std::vector<cv::Point2f>& points0_return,
                                std::vector<uchar>& status0, std::vector<uchar>& status1,
                                std::vector<uchar>& status2, std::vector<uchar>& status3)
        {
            int j = 0;
            for( int i=0; i<status3.size(); i++)
            {  
                cv::Point2f p0 = points0.at(i - j);
                cv::Point2f p1 = points1.at(i - j);
                cv::Point2f p2 = points2.at(i - j);
                cv::Point2f p3 = points3.at(i - j);
                cv::Point2f p0_r = points0_return.at(i- j);
                
                if ((status0.at(i) == 0) ||(status1.at(i) == 0)||
                    (status3.at(i) == 0) ||(status2.at(i) == 0)||
                    (p0.x<0) || (p0.y<0) || (p1.x<0) || (p1.y<0) ||
                    (p2.x<0) || (p2.y<0) || (p3.x<0) || (p3.y<0))        
                {

                    points0.erase (points0.begin() + (i - j));
                    points1.erase (points1.begin() + (i - j));
                    points2.erase (points2.begin() + (i - j));
                    points3.erase (points3.begin() + (i - j));
                    points0_return.erase (points0_return.begin() + (i - j));

                    j++;
                }

            }  
        }
        void circularMatching(cv::Mat img_l_0, cv::Mat img_r_0, cv::Mat img_l_1, cv::Mat img_r_1,
                      std::vector<cv::Point2f>& points_l_0, std::vector<cv::Point2f>& points_r_0,
                      std::vector<cv::Point2f>& points_l_1, std::vector<cv::Point2f>& points_r_1,
                      std::vector<cv::Point2f>& points_l_0_return) 
        { 
        
            /*Track the features using LK optical flow sequentially through left and right images and in prev and curr timestep.
            Then the deleteUntrackedFeatures function removes all the points which didnt track in any of the steps
            */

            std::vector<float> err;                    
            cv::Size winSize=cv::Size(21,21);                                                                                             
            cv::TermCriteria termcrit=cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);

            std::vector<uchar> status0;
            std::vector<uchar> status1;
            std::vector<uchar> status2;
            std::vector<uchar> status3;

            calcOpticalFlowPyrLK(img_l_0, img_r_0, points_l_0, points_r_0, status0, err, winSize, 3, termcrit, 0, 0.001);
            calcOpticalFlowPyrLK(img_r_0, img_r_1, points_r_0, points_r_1, status1, err, winSize, 3, termcrit, 0, 0.001);
            calcOpticalFlowPyrLK(img_r_1, img_l_1, points_r_1, points_l_1, status2, err, winSize, 3, termcrit, 0, 0.001);
            calcOpticalFlowPyrLK(img_l_1, img_l_0, points_l_1, points_l_0_return, status3, err, winSize, 3, termcrit, 0, 0.001);

            deleteUntrackedFeatures(points_l_0, points_r_0,
                                    points_r_1, points_l_1,
                                    points_l_0_return,
                                    status0, status1,
                                    status2, status3);
            // std::cout << "points : " << points_l_0.size() << " "<< points_r_0.size() << " "<< points_r_1.size() << " "<< points_l_1.size() << " "<<std::endl;
        }
        void checkValidMatch(std::vector<cv::Point2f>& points, std::vector<cv::Point2f>& points_return, std::vector<bool>& status, int threshold)
        { /* Check if a tracked point is usable: if two points have a mismatch larger than a threshold they are deemed invalid
            */
            int offset;
            for (int i = 0; i < points.size(); i++)
            {
                offset = std::max(std::abs(points[i].x - points_return[i].x), std::abs(points[i].y - points_return[i].y));

                if(offset > threshold)
                {
                    status.push_back(false);
                }
                else
                {
                    status.push_back(true);
                }
            }
        }

        void removeInvalidPoints(std::vector<cv::Point2f>& points, const std::vector<bool>& status)
        {
            int index = 0;
            for (int i = 0; i < status.size(); i++)
            {
                if (status[i] == false)
                {
                    points.erase(points.begin() + index);
                }
                else
                {
                    index ++;
                }
            }
        }

        void matchAndTrack(cv::Mat& image_l_0, cv::Mat& image_r_0,
                        cv::Mat& image_l_1, cv::Mat& image_r_1, 
                        std::vector<cv::Point2f>&  points_l_0, 
                        std::vector<cv::Point2f>&  points_r_0, 
                        std::vector<cv::Point2f>&  points_l_1, 
                        std::vector<cv::Point2f>&  points_r_1
                        )
        {

            std::vector<cv::Point2f>  points_l_0_return; 

            circularMatching(image_l_0, image_r_0,
                            image_l_1, image_r_1,
                            points_l_0,
                            points_r_0, 
                            points_l_1, 
                            points_r_1,
                            points_l_0_return);
            
            std::vector<bool> status;
            
            checkValidMatch(points_l_0, points_l_0_return, status, 0);

            removeInvalidPoints(points_l_0, status);
            removeInvalidPoints(points_l_1, status);
            removeInvalidPoints(points_r_0, status);
            removeInvalidPoints(points_r_1, status);

            // visualizeTracking(image_tracking, image_l_1, points_l_0, points_l_1);
        }

        


        cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R)
        {
        
            assert(isRotationMatrix(R));
            
            float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
        
            bool singular = sy < 1e-6; // If
        
            float x, y, z;
            if (!singular)
            {
                x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
                y = atan2(-R.at<double>(2,0), sy);
                z = atan2(R.at<double>(1,0), R.at<double>(0,0));
            }
            else
            {
                x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
                y = atan2(-R.at<double>(2,0), sy);
                z = 0;
            }
            return cv::Vec3f(x, y, z);
            
        }

        void integrateOdometry(int frame_i, cv::Mat& rigid_body_transformation, cv::Mat& frame_pose, const cv::Mat& rotation, const cv::Mat& translation_stereo)
        {

            
            cv::Mat addup = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);

            cv::hconcat(rotation, translation_stereo, rigid_body_transformation);
            cv::vconcat(rigid_body_transformation, addup, rigid_body_transformation);

            double scale = sqrt((translation_stereo.at<double>(0))*(translation_stereo.at<double>(0)) 
                                + (translation_stereo.at<double>(1))*(translation_stereo.at<double>(1))
                                + (translation_stereo.at<double>(2))*(translation_stereo.at<double>(2))) ;

            rigid_body_transformation = rigid_body_transformation.inv();
            
            // if ((scale>0.1)&&(translation_stereo.at<double>(2) > translation_stereo.at<double>(0)) && (translation_stereo.at<double>(2) > translation_stereo.at<double>(1))) 
            if (scale > 0.05 && scale < 10) 
            {

            frame_pose = frame_pose * rigid_body_transformation;

            }
            else 
            {
            std::cout << "[WARNING] scale is very low or very high" << std::endl;
            }
        }

};

struct SnavelyReprojectionError
{
    SnavelyReprojectionError(double focal, double cx, double cy,
                             double observed_x, double observed_y)
        : focal_(focal), cx_(cx), cy_(cy),
          observed_x_(observed_x), observed_y_(observed_y){}

    template<typename T>
    bool operator()(const T* const cam_pose, const T* const point, T* residuals) const {
        // camera[0,1,2] are the angle-axis rotation.
        T p[3];

        // y = R(angle_axis) * x;
        ceres::AngleAxisRotatePoint(cam_pose, point, p);
        
        p[0] += cam_pose[3];
        p[1] += cam_pose[4];
        p[2] += cam_pose[5];

        const T xp = p[0]/p[2];
        const T yp = p[1]/p[2];

        // Apply second and fourth order radial distortion.
        // const T& l1 = camera[7];
        // const T& l2 = camera[8];
        // const T r2 = xp * xp + yp * yp;
        // const T distortion = 1.0 + r2 * (l1 + l2 * r2);

        // Compute final projected point position. -> prediction의 pixel coordinate
        // const T& focal = camera[6];
        const T predicted_x = focal_ * xp + cx_;
        const T predicted_y = focal_ * yp + cy_;

        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_x - observed_x_;
        residuals[1] = predicted_y - observed_y_;

        return true;
    }
    
    double observed_x_;
    double observed_y_;
    double focal_, cx_, cy_;
};

const std::string imageExtensions[] = {".jpg", ".jpeg", ".png", ".gif", ".bmp"};
inline void CountImages(int &num_images, const std::string &path)
{
    try
    {
        // 지정된 폴더 내의 모든 파일에 대해 반복
        for (const auto &entry : boost::filesystem::directory_iterator(path))
        {
            // 디렉토리인 경우 건너뛰기
            if (boost::filesystem::is_directory(entry.path()))
                continue;

            // 이미지 파일인 경우 개수 증가
            for (const std::string &ext : imageExtensions)
            {
                if (entry.path().extension() == ext)
                    num_images++;
            }
        }

        std::cout << "Number of image files in the folder: " << num_images << std::endl;
    }
    catch (const std::exception &ex)
    {
        std::cerr << "Error: " << ex.what() << std::endl;
    }
}
inline void readGTPose(const std::string& path, std::vector<cv::Mat>& gt_poses)
{
    std::ifstream file(path);
    std::string line, word;

    if(file.is_open())
    {
        while(getline(file, line))
        {
            int i = 0, j = 0;

            std::stringstream ss(line);
            cv::Mat gt_pose = cv::Mat::eye(4, 4, CV_64F);
            
            while(getline(ss, word, ' '))
            {
                gt_pose.at<double>(j,i) = std::stod(word);

                i++;
                if(i==4)
                {
                    i=0;
                    j++;
                }
            }
            gt_poses.push_back(gt_pose);
        }
        file.close();
    }
    else
    {
        std::cout << "file not found" << std::endl;
        exit(0);
    }
}
inline double calculateRotationError(const cv::Mat& R_gt, const cv::Mat& R_est) {
    // Calculate the difference rotation matrix
    cv::Mat R_diff = R_gt.t() * R_est;

    // Trace of R_diff
    double trace = R_diff.at<double>(0, 0) + R_diff.at<double>(1, 1) + R_diff.at<double>(2, 2);

    // Calculate the rotation error in radians
    // axis-angle roation(rodrigues) error
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
inline void visualizeRelative3D(cv::viz::Viz3d& myWindow, const Node& refer, const Node& query, const cv::Mat gt_refer_pose, const cv::Mat gt_query_pose, const int& idx)
{   
    cv::Affine3f world_pose, gt_curr_pose, esti_curr_pose;
    cv::Mat gt_diff, esti_diff;
    gt_diff = gt_refer_pose.inv() * gt_query_pose;
    esti_diff = refer.left_cam.cam_to_world_pose.inv() * query.left_cam.cam_to_world_pose;

    gt_curr_pose.rotation(cv::Mat_<float>(gt_diff(cv::Rect(0,0,3,3))));
    gt_curr_pose.translation(cv::Mat_<float>(gt_diff.rowRange(0,3).colRange(3,4)));
    esti_curr_pose.rotation(cv::Mat_<float>(esti_diff(cv::Rect(0,0,3,3))));
    esti_curr_pose.translation(cv::Mat_<float>(esti_diff.rowRange(0,3).colRange(3,4)));

    cv::viz::WText3D world_text("world", cv::Point3d(0,0,0), 0.1, true, cv::viz::Color::black());
    cv::viz::WText3D gt_text("gt", cv::Point3d(gt_curr_pose.translation()(0),gt_curr_pose.translation()(1),gt_curr_pose.translation()(2)), 0.1, true, cv::viz::Color::red());
    cv::viz::WText3D esti_text("esti", cv::Point3d(esti_curr_pose.translation()(0),esti_curr_pose.translation()(1),esti_curr_pose.translation()(2)), 0.1, true, cv::viz::Color::cyan());

    // for(int i = 0; i < refer.points3D.rows; i++)
    // {
    //     cv::Mat point = (cv::Mat_<double>(4,1) << refer.points3D.row(i).at<float>(0),
    //                                               refer.points3D.row(i).at<float>(1),
    //                                               refer.points3D.row(i).at<float>(2),
    //                                               1.0);
    //     cv::Mat world_point = query.left_cam.world_to_cam_pose * point;
    //     cv::viz::WSphere point_wiz(cv::Point3d(world_point.at<double>(0), world_point.at<double>(1), world_point.at<double>(2)),
    //                               0.1, 10, cv::viz::Color::red());
    //     myWindow.showWidget("point" + std::to_string(i), point_wiz);
    // }



    myWindow.showWidget("world_text", world_text);
    myWindow.showWidget("gt_text", gt_text);
    myWindow.showWidget("esti_text", esti_text);
    myWindow.showWidget(std::to_string(0), cv::viz::WCoordinateSystem(), world_pose);
    myWindow.showWidget("gt" + std::to_string(idx-1), cv::viz::WCoordinateSystem(), gt_curr_pose);
    myWindow.showWidget("esti" + std::to_string(idx-1), cv::viz::WCoordinateSystem(), esti_curr_pose);
    myWindow.spinOnce(10, false);
    myWindow.removeAllWidgets();
}