//https://github.com/FilipposSot/stereo-vo/tree/main

#include "3DVision-tutor/stereo_vo.hpp"

int main(int argc, char **argv)
{
    double focal = 718.8560;
    cv::Point2d pp(607.1928, 185.2157);
    cv::Mat projMat_l = (cv::Mat_<float>(3,4) << 7.188560000000e+02, 0.000000000000e+00, 6.071928000000e+02, 4.538225000000e+01,
                                                0.000000000000e+00, 7.188560000000e+02, 1.852157000000e+02, -1.130887000000e-01,
                                                0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 3.779761000000e-03);
    cv::Mat projMat_r = (cv::Mat_<float>(3,4) << 7.188560000000e+02, 0.000000000000e+00, 6.071928000000e+02, -3.372877000000e+02,
                                                0.000000000000e+00, 7.188560000000e+02, 1.852157000000e+02, 2.369057000000e+00,
                                                0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 4.915215000000e-03);
    
    cv::Mat traj = cv::Mat::zeros(1000, 1000, CV_8UC3);

    cv::Mat rotation = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat translation = cv::Mat::zeros(3, 1, CV_64F);

    cv::Mat pose = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat Rpose = cv::Mat::eye(3, 3, CV_64F);
    
    cv::Mat frame_pose = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat frame_pose32 = cv::Mat::eye(4, 4, CV_32F);

    cv::Mat currImage_l, currImage_r;
    cv::Mat prevImage_l, prevImage_r;

    std::string left_path = "/home/cona/Downloads/dataset/data_odometry_gray/dataset/sequences/00/image_0/";
    std::string right_path = "/home/cona/Downloads/dataset/data_odometry_gray/dataset/sequences/00/image_1/";

    int num_images = 0;
    CountImages(num_images, left_path);

    cv::Mat prevImage_c_l = cv::imread(left_path + cv::format("%06d.png", 0));
    cv::Mat prevImage_c_r = cv::imread(right_path + cv::format("%06d.png", 0));

    cv::cvtColor(prevImage_c_l, prevImage_l, cv::COLOR_BGR2GRAY);
    cv::cvtColor(prevImage_c_r, prevImage_r, cv::COLOR_BGR2GRAY);
    //--------------------------------------------------------------------------------------------------------------//
    
    cv::Mat inliers;
    cv::Mat points3D_t0, points4D_t0;
    

    for(int numFrame = 1; numFrame < num_images; numFrame++)
    {
        std::vector<cv::Point3f> points3d;
        std::vector<cv::Point2f> prevPoints_l;

        cv::Mat currImage_c_l = cv::imread(left_path + cv::format("%06d.png", numFrame));
        cv::Mat currImage_c_r = cv::imread(right_path + cv::format("%06d.png", numFrame));
        cv::cvtColor(currImage_c_l, currImage_l, cv::COLOR_BGR2GRAY);
        cv::cvtColor(currImage_c_r, currImage_r, cv::COLOR_BGR2GRAY);

        if(prevPoints_l.size() == 0)
            featureDetectionFAST(prevImage_l, prevPoints_l);
        // if (prevPoints_l.size() < 2000) {
        //   appendNewPoints(prevImage_c_l, prevPoints_l);
        // }

        std::vector<cv::Point2f> prevPoints_r, currPoints_l, currPoints_r;
        cv::Mat image_tracking;
        matchAndTrack( prevImage_l, prevImage_r,
                       currImage_l, currImage_r, 
                       prevPoints_l, 
                       prevPoints_r, 
                       currPoints_l, 
                       currPoints_r,
                       image_tracking);
        
        cv::triangulatePoints( projMat_l,  projMat_r,  currPoints_l, currPoints_r,  points4D_t0);
        cv::convertPointsFromHomogeneous(points4D_t0.t(), points3D_t0);
        // Mat2Vec(points3D_t0, points3d);
        odometryCalculation(projMat_l, projMat_r, 
                            prevPoints_l, currPoints_l,
                            points3D_t0, rotation, translation, inliers);

        // clone images and points for the next iteration
        prevImage_l = currImage_l.clone();
        prevImage_r = currImage_r.clone();
        prevImage_c_l = currImage_c_l.clone();
        prevImage_c_r = currImage_c_r.clone();
        prevPoints_l  = currPoints_l;

        cv::Vec3f rotation_euler = rotationMatrixToEulerAngles(rotation);
        cv::Mat rigid_body_transformation;

        // if the rotation is reasonable integrate the visual odometry to the current pose
        if(abs(rotation_euler[1])<0.1 && abs(rotation_euler[0])<0.1 && abs(rotation_euler[2])<0.1)
        {
            integrateOdometry(0 , rigid_body_transformation, frame_pose, rotation, translation);

        } else {

            std::cout << "Too large rotation"  << std::endl;
        }

        visualizeTrajectory(traj, frame_pose, points3D_t0, inliers );

        cv::waitKey(1);
    }

    return 0;
}