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

void featureDetectionFAST(cv::Mat img, std::vector<cv::Point2f>& points)
{
    /* Detect features and return the image point for a single image using OpenCV FAST implementation*/
    std::vector<cv::KeyPoint> keypoints;
    int fast_threshold = 40;
    bool nonmaxSuppression = true;
    cv::FAST(img, keypoints, fast_threshold, nonmaxSuppression);
    cv::KeyPoint::convert(keypoints, points, std::vector<int>());
}

void Mat2Vec(const cv::Mat& points3D_t0, std::vector<cv::Point3f>& points3d)
{
    for(int i = 0; i < points3D_t0.rows; i++)
    {
        points3d.push_back(cv::Point3f(points3D_t0.row(i).at<float>(0, 0),
                                       points3D_t0.row(i).at<float>(0, 1),
                                       points3D_t0.row(i).at<float>(0, 2)));
    }
}
void appendNewPoints(cv::Mat& image, std::vector<cv::Point2f>& points)
{/* Add a new set of point to a vector with some already used points
    */
    std::vector<cv::Point2f>  points_new;
    featureDetectionFAST(image, points_new);
    points.insert(points.end(), points_new.begin(), points_new.end());
}

void visualizeTracking( cv::Mat& vis,
                        cv::Mat& image_l_1,
                        std::vector<cv::Point2f>&  points_l_0, 
                        std::vector<cv::Point2f>&  points_l_1){

    int radius = 2;
    cvtColor(image_l_1, vis, cv::COLOR_GRAY2BGR, 3);

    for (int i = 0; i < points_l_0.size(); i++)
        {
          cv::circle(vis, cv::Point(points_l_0[i].x, points_l_0[i].y), radius, CV_RGB(0,255,0));
        }

    for (int i = 0; i < points_l_1.size(); i++)
        {
          cv::circle(vis, cv::Point(points_l_1[i].x, points_l_1[i].y), radius, CV_RGB(0,0,255));
        }

    for (int i = 0; i < points_l_1.size(); i++)
        {
          cv::line(vis, points_l_0[i], points_l_1[i], CV_RGB(0,255,0));
        }

      cv::imshow("vis ", vis );
}

void odometryCalculation(cv::Mat& projMatrl, cv::Mat& projMatrr,
                         std::vector<cv::Point2f>&  pointsLeft_t0,
                         std::vector<cv::Point2f>&  pointsLeft_t1, 
                         cv::Mat& points3D_t0,
                         cv::Mat& rotation,
                         cv::Mat& translation,
                         cv::Mat& inliers)
{

      double focal = projMatrl.at<float>(0, 0);
      cv::Point2d principle_point(projMatrl.at<float>(0, 2), projMatrl.at<float>(1, 2));

      cv::Mat distCoeffs       = cv::Mat::zeros(4, 1, CV_64FC1);   
      cv::Mat rvec             = cv::Mat::zeros(3, 1, CV_64FC1);
      cv::Mat intrinsic_matrix = (cv::Mat_<float>(3, 3) << projMatrl.at<float>(0, 0), projMatrl.at<float>(0, 1), projMatrl.at<float>(0, 2),
                                                           projMatrl.at<float>(1, 0), projMatrl.at<float>(1, 1), projMatrl.at<float>(1, 2),
                                                           projMatrl.at<float>(2, 0), projMatrl.at<float>(2, 1), projMatrl.at<float>(2, 2));
                                                   // projMatrl.at<float>(1, 1), projMatrl.at<float>(1, 2), projMatrl.at<float>(1, 3));

      int iterationsCount = 500;        // number of Ransac iterations.
      float reprojectionError = 0.1;    // maximum allowed distance to consider it an inlier.
      float confidence = 0.999;         // RANSAC successful confidence.
      bool useExtrinsicGuess = true;
      int flags =cv::SOLVEPNP_ITERATIVE;
      
      cv::solvePnPRansac( points3D_t0, pointsLeft_t1, intrinsic_matrix, distCoeffs, rvec, translation,
                          useExtrinsicGuess, iterationsCount, reprojectionError, confidence,
                          inliers, flags );
      
      cv::Rodrigues(rvec, rotation);

      // std::cout << "[trackingFrame2Frame] inliers size: " << inliers.size() << std::endl;

}

void deleteUntrackedFeatures(   std::vector<cv::Point2f>& points0, std::vector<cv::Point2f>& points1,
                                std::vector<cv::Point2f>& points2, std::vector<cv::Point2f>& points3,
                                std::vector<cv::Point2f>& points0_return,
                                std::vector<uchar>& status0, std::vector<uchar>& status1,
                                std::vector<uchar>& status2, std::vector<uchar>& status3){

  int j = 0;
  for( int i=0; i<status3.size(); i++)
     {  cv::Point2f p0 = points0.at(i - j);
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
                   std::vector<cv::Point2f>&  points_r_1,
                   cv::Mat& image_tracking){

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

    visualizeTracking(image_tracking, image_l_1, points_l_0, points_l_1);

}

void removeInvDepth(std::vector<cv::Point2f>&  points_l_0, 
                   std::vector<cv::Point2f>&  points_r_0, 
                   std::vector<cv::Point2f>&  points_l_1, 
                   std::vector<cv::Point2f>&  points_r_1,
                   cv::Mat& points3D_t0)
{
    std::vector<bool> status;
    cv::Mat tmp;
    for(int i = 0; i < points3D_t0.rows; i++)
    {
        if(points3D_t0.at<float>(i,2) < 0)
            status.push_back(false);
        else
        {
            status.push_back(true);
            tmp.push_back(points3D_t0.row(i));
        }
    }
    removeInvalidPoints(points_l_0, status);
    removeInvalidPoints(points_l_1, status);
    removeInvalidPoints(points_r_0, status);
    removeInvalidPoints(points_r_1, status);
    points3D_t0 = tmp.clone();
}

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


cv::Mat readGTPose(const std::string& path, int index)
{
    std::ifstream file(path);
    std::string line, word;
    cv::Mat gt_pose = cv::Mat::eye(4, 4, CV_64F);
    int count = 0;

    if(file.is_open())
    {
        while(getline(file, line))
        {
            int i = 0, j = 0;

            std::stringstream ss(line);
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
            if(count == index)
                break;
            count++;
        }
        file.close();
        return gt_pose;
    }
    else
    {
        std::cout << "file not found" << std::endl;
        exit(0);
    }
};

void visualizeTrajectory(cv::Mat& traj,
                         cv::Mat& frame_pose,
                         const cv::Mat& points3D_t0,
                         const cv::Mat& inliers,
                         const cv::Scalar& traj_color){
    
    int x = int(frame_pose.at<double>(0,3)) + 500;
    int y = int(frame_pose.at<double>(2,3)) + 400;
    
    cv::Mat addup = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);
    cv::Mat I = (cv::Mat_<double>(3, 3) << 1,0,0,
                                          0,1,0,
                                          0,0,1);

    cv::Mat rigid_body_transformation;
    cv::Mat points3D_world ;
    
    // cout << "Number of Inliers: " << inliers.size() << endl;

    int j;

    for (int i = 0; i < inliers.size().height; i++){
        j = inliers.at<int>(0, i);
        
        // cout << i << " " << j  << endl;

        cv::Mat translation = (cv::Mat_<double>(3, 1) << points3D_t0.row(j).at<float>(0, 0),
                                                        points3D_t0.row(j).at<float>(0, 1),
                                                        points3D_t0.row(j).at<float>(0, 2));

        cv::hconcat(I, translation, rigid_body_transformation);
        cv::vconcat(rigid_body_transformation, addup, rigid_body_transformation);
        points3D_world =  frame_pose*rigid_body_transformation;

        int x_point = int(points3D_world.at<double>(0,3)) + 500;
        int y_point = int(points3D_world.at<double>(2,3)) + 400;

        cv::circle(traj, cv::Point(x_point, y_point) ,0.5, CV_RGB(0,100,100), 0.5);

    }



    cv::circle(traj, cv::Point(x, y) ,1, traj_color, 2);
    // rectangle( traj, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), cv::FILLED);

    cv::imshow( "Trajectory", traj );
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

bool isRotationMatrix(cv::Mat &R)
{
    cv::Mat Rt;
    transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());
     
    return  norm(I, shouldBeIdentity) < 1e-6;
     
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