//https://github.com/FilipposSot/stereo-vo/tree/main

#include "3DVision-tutor/stereo_vo.hpp"

int main(int argc, char **argv)
{
    double focal = 718.8560;
    cv::Point2d pp(607.1928, 185.2157);
    cv::Mat projMat_l = (cv::Mat_<float>(3,4) << 7.188560000000e+02, 0.000000000000e+00, 6.071928000000e+02, 0.000000000000e+00,
                                                0.000000000000e+00, 7.188560000000e+02, 1.852157000000e+02, 0.000000000000e+00,
                                                0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 0.000000000000e+00);
    cv::Mat projMat_r = (cv::Mat_<float>(3,4) << 7.188560000000e+02, 0.000000000000e+00, 6.071928000000e+02, -3.861448000000e+02,
                                                0.000000000000e+00, 7.188560000000e+02, 1.852157000000e+02, 0.000000000000e+00,
                                                0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 0.000000000000e+00);
    
    cv::Mat traj = cv::Mat::zeros(1000, 1000, CV_8UC3);

    cv::Mat rotation = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat translation = cv::Mat::zeros(3, 1, CV_64F);

    cv::Mat pose = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat Rpose = cv::Mat::eye(3, 3, CV_64F);
    
    cv::Mat frame_pose = cv::Mat::eye(4, 4, CV_64F); //world 기준 camera pose
    cv::Mat gt_pose = cv::Mat::eye(4, 4, CV_64FC1);
    cv::Mat prev_pose;

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
    // cv::VideoWriter video_trajectory("trajectory.avi", cv::VideoWriter::fourcc('M','J','P','G'), 25, cv::Size(1000,1000));
    // cv::VideoWriter video_tracking("tracking.avi"  , cv::VideoWriter::fourcc('M','J','P','G'), 25, prevImage_c_l.size());

    // cv::viz::Viz3d myWindow("Coordinate Frame");
    // myWindow.setWindowSize(cv::Size(640,480));

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
        
        cv::triangulatePoints( projMat_l,  projMat_r,  prevPoints_l, prevPoints_r,  points4D_t0);
        cv::convertPointsFromHomogeneous(points4D_t0.t(), points3D_t0);
        // Mat2Vec(points3D_t0, points3d);

        // removeInvDepth(prevPoints_l, 
        //                prevPoints_r, 
        //                currPoints_l, 
        //                currPoints_r,
        //                points3D_t0);

        // relative pose 계산
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
        cv::Mat rigid_body_transformation; //relative pose

        // if the rotation is reasonable integrate the visual odometry to the current pose
        if(abs(rotation_euler[1])<0.1 && abs(rotation_euler[0])<0.1 && abs(rotation_euler[2])<0.1)
        {
            integrateOdometry(0 , rigid_body_transformation, frame_pose, rotation, translation);

        } else {

            std::cout << "Too large rotation"  << std::endl;
        }

        visualizeTrajectory(traj, frame_pose, points3D_t0, inliers, cv::Scalar(255, 255, 0));

        //GT
        prev_pose = gt_pose.clone();
        gt_pose = readGTPose("/home/cona/Downloads/dataset/data_odometry_gray/data_odometry_poses/dataset/poses/00.txt", numFrame);
        visualizeTrajectory(traj, gt_pose, cv::Mat(), cv::Mat(), cv::Scalar(0, 0, 255) );

        // //calculate error
        double r_err, t_err;
        cv::Mat gt_diff = prev_pose.inv()*gt_pose;
        r_err = calculateRotationError(gt_diff(cv::Rect(0,0,3,3)), rigid_body_transformation(cv::Rect(0,0,3,3)));
        t_err = calculateTranslationError(gt_diff.rowRange(0,3).colRange(3,4), rigid_body_transformation.rowRange(0,3).colRange(3,4));
        std::cout << "frame" << numFrame-1 << ", frame" << numFrame << std::endl;
        std::cout << "rot_error(degree): " << r_err << std::endl;
        std::cout << "tr_error(m): " << t_err << std::endl;
        std::cout << "" << std::endl;

        // video_trajectory.write(traj);
        // video_tracking.write(image_tracking);

        //visualize 3d pose
        // cv::Affine3f world_pose, gt_curr_pose, esti_curr_pose;
        // world_pose.rotation(cv::Mat::eye(3,3,CV_32F));
        // world_pose.translation(cv::Vec3f(0,0,0));
        // gt_curr_pose.rotation(cv::Mat_<float>(gt_diff(cv::Rect(0,0,3,3))));
        // gt_curr_pose.translation(cv::Mat_<float>(gt_diff.rowRange(0,3).colRange(3,4)));
        // esti_curr_pose.rotation(cv::Mat_<float>(rigid_body_transformation(cv::Rect(0,0,3,3))));
        // esti_curr_pose.translation(cv::Mat_<float>(rigid_body_transformation.rowRange(0,3).colRange(3,4)));
    
        // cv::viz::WText3D world_text("world", cv::Point3d(0,0,0), 0.1, true, cv::viz::Color::black());
        // cv::viz::WText3D gt_text("gt", cv::Point3d(gt_curr_pose.translation()(0),gt_curr_pose.translation()(1),gt_curr_pose.translation()(2)), 0.1, true, cv::viz::Color::red());
        // cv::viz::WText3D esti_text("esti", cv::Point3d(esti_curr_pose.translation()(0),esti_curr_pose.translation()(1),esti_curr_pose.translation()(2)), 0.1, true, cv::viz::Color::cyan());

        // myWindow.showWidget("world_text", world_text);
        // myWindow.showWidget("gt_text", gt_text);
        // myWindow.showWidget("esti_text", esti_text);
        // myWindow.showWidget(std::to_string(0), cv::viz::WCoordinateSystem(), world_pose);
        // myWindow.showWidget("gt" + std::to_string(numFrame-1), cv::viz::WCoordinateSystem(), gt_curr_pose);
        // myWindow.showWidget("esti" + std::to_string(numFrame-1), cv::viz::WCoordinateSystem(), esti_curr_pose);
        // myWindow.spinOnce(500, false);
        // myWindow.removeAllWidgets();
        cv::waitKey(1);
    }

    return 0;
}