#include "3DVision-tutor/motion_only_ba.hpp"

int main(int argc, char **argv)
{
    // std::string left_path = "/home/cona/Downloads/dataset/data_odometry_gray/dataset/sequences/00/image_0/";
    // std::string right_path = "/home/cona/Downloads/dataset/data_odometry_gray/dataset/sequences/00/image_1/";
    std::string left_path = "/home/cona/yhj/data/bk2/left/";
    std::string right_path = "/home/cona/yhj/data/bk2/right/";
    std::vector<cv::Mat> gt_poses;
    int num_images = 0;
    CountImages(num_images, left_path);
    readGTPose("/home/cona/Downloads/dataset/data_odometry_gray/data_odometry_poses/dataset/poses/00.txt", gt_poses);
    cv::viz::Viz3d myWindow("Coordinate Frame");
    myWindow.setWindowSize(cv::Size(640,480));

    Tracker tracker;
    Visualizer visualizer;

    for(int i = 0; i < num_images-1; i++)
    {
        ceres::Problem problem;
        Node refer, query;
        cv::Mat viz_track, viz_extract;

        refer.left_cam.loadImage(left_path + cv::format("%06d.png", i));
        refer.right_cam.loadImage(right_path + cv::format("%06d.png", i));
        query.left_cam.loadImage(left_path + cv::format("%06d.png", i+1));
        query.right_cam.loadImage(right_path + cv::format("%06d.png", i+1));
        
        tracker.extractKeypoints(refer.left_cam);
        tracker.trackKeypoints(refer, query);
        tracker.calc3DPoints(refer);
        tracker.calcPose(refer, query);
        
        // refer.converDouble();
        // query.converDouble();

        // if(i==0)
        //     cv::hconcat(refer.left_cam.rot_rodrigues.t(), refer.left_cam.translation.t(),refer.left_cam.ceres_cam_pose);

        // for(int j = 0; j < refer.left_cam.keypoints.size(); j++)
        // {
        //     ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
        //                                         new SnavelyReprojectionError(refer.left_cam.focal,
        //                                                                     refer.left_cam.pp.x, refer.left_cam.pp.y,
        //                                                                     refer.left_cam.keypoints[j].x, refer.left_cam.keypoints[j].y));
        //     problem.AddResidualBlock(cost_function, NULL,
        //                             &refer.left_cam.ceres_cam_pose[0],
        //                             &refer.landmarks[j*3]);
                      
        //     problem.SetParameterBlockConstant(&refer.left_cam.ceres_cam_pose[0]);

        //     ceres::CostFunction* cost_function2 = new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
        //                                         new SnavelyReprojectionError(refer.right_cam.focal,
        //                                                                     refer.right_cam.pp.x, refer.right_cam.pp.y,
        //                                                                     refer.right_cam.keypoints[j].x, refer.right_cam.keypoints[j].y));
        //     problem.AddResidualBlock(cost_function, NULL,
        //                             &refer.right_cam.ceres_cam_pose[0],
        //                             &refer.landmarks[j*3]);

        //     // ceres::CostFunction* cost_function2 = new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
        //     //                                     new SnavelyReprojectionError(query.left_cam.focal,
        //     //                                                                 query.left_cam.pp.x, query.left_cam.pp.y,
        //     //                                                                 query.left_cam.keypoints[j].x, query.left_cam.keypoints[j].y));
        //     // problem.AddResidualBlock(cost_function2, NULL,
        //     //                         &query.left_cam.ceres_cam_pose[0],
        //     //                         &refer.landmarks[j*3]);
        // }
        
        // ceres::Solver::Options options;
        // options.linear_solver_type = ceres::SPARSE_SCHUR;
        // options.minimizer_progress_to_stdout = true;
        // options.max_num_iterations = 100;
        // ceres::Solver::Summary summary;
        // ceres::Solve(options, &problem, &summary);

        // std::cout << summary.FullReport() << std::endl;
        
        // //calculate error
        // double r_err, t_err;
        // cv::Mat gt_diff = gt_poses[i].inv()*gt_poses[i+1];
        // r_err = calculateRotationError(gt_diff(cv::Rect(0,0,3,3)), query.left_cam.cam_to_world_pose(cv::Rect(0,0,3,3)));
        // t_err = calculateTranslationError(gt_diff.rowRange(0,3).colRange(3,4), query.left_cam.cam_to_world_pose.rowRange(0,3).colRange(3,4));
        // std::cout << "frame" << i << ", frame" << i+1 << std::endl;
        // std::cout << "rot_error(degree): " << r_err << std::endl;
        // std::cout << "tr_error(m): " << t_err << std::endl;
        
        std::cout << "frame" << i << " -> frame" << i+1 << std::endl; 
        std::cout << query.left_cam.cam_to_world_pose << std::endl;
        std::cout << "" << std::endl;
        
        visualizer.visualizeExtract(refer, viz_extract);
        visualizer.visualizeTracking(refer, query, viz_track);
        visualizer.visualizeRelative3D(myWindow, refer, query, gt_poses[i], gt_poses[i+1], i);
        cv::waitKey(0);
    }
    
    return 0;
}