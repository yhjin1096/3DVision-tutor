#include "3DVision-tutor/comparison.hpp"

int main(int argc, char **argv)
{
    // std::string left_path = "/home/cona/Downloads/dataset/data_odometry_gray/dataset/sequences/00/image_0/";
    // std::string right_path = "/home/cona/Downloads/dataset/data_odometry_gray/dataset/sequences/00/image_1/";
    std::string left_path = "/home/cona/yhj/data/bk3_HD/left/";
    std::string right_path = "/home/cona/yhj/data/bk3_HD/right/";
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
        Node refer, query;

        refer.left_cam.loadImage(left_path + cv::format("%06d.png", i));
        refer.right_cam.loadImage(right_path + cv::format("%06d.png", i));
        query.left_cam.loadImage(left_path + cv::format("%06d.png", i+1));
        query.right_cam.loadImage(right_path + cv::format("%06d.png", i+1));
        
        // tracker.extractKeypointsFAST(refer.left_cam); //FAST
        tracker.extractKeypointsORB(refer.left_cam); //ORB
        tracker.extractKeypointsORB(refer.right_cam); //ORB
        // tracker.extractKeypointsStella(refer.left_cam); //Stella
        // tracker.extractKeypointsStella(refer.right_cam); //Stella
        
        tracker.trackKyepointsWDescriptor(refer, query);
        // tracker.trackKeypointsCircular(refer, query);
        
        tracker.calc3DPoints(refer);
        tracker.calcPose(refer, query);
        
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
        
        cv::Mat viz_track, viz_extract;
        visualizer.visualizeExtractAll(refer, query, viz_extract);
        visualizer.visualizeTracking(refer, query, viz_track);
        visualizer.visualizeRelative3D(myWindow, refer, query, gt_poses[i], gt_poses[i+1], i);
        cv::waitKey(0);
    }
    
    return 0;
}