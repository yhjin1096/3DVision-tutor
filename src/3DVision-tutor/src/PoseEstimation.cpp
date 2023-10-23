#include <3DVision-tutor/common.hpp>

class Node
{
    public:
        Camera left_cam, right_cam;

        Node()
        {
            //오른쪽 카메라 기준 왼쪽 카메라 위치
            right_cam.pose_eig.t << -right_cam.base_line, 0.0, 0.0;
            left_cam.pose_aff = Calculator::Eig_to_Aff(left_cam.pose_eig);
            right_cam.pose_aff = Calculator::Eig_to_Aff(right_cam.pose_eig);
        }

        void setCameraPose(const pose_t& pose)
        {
            //world <-> camera
            this->left_cam.pose_eig.R = pose.R.inverse();
            this->left_cam.pose_eig.t = -pose.R.inverse()*pose.t;
            this->left_cam.pose_aff = Calculator::Eig_to_Aff(this->left_cam.pose_eig);
            
            //world <-> left camera <-> right camera
            this->right_cam.pose_eig.R = pose.R.inverse();
            this->right_cam.pose_eig.t = this->right_cam.pose_eig.R * this->left_cam.pose_eig.t + this->right_cam.pose_eig.t;
            this->right_cam.pose_aff = Calculator::Eig_to_Aff(this->right_cam.pose_eig);
        };

    private:
};

int PoseEstimation_Ex1()
{
    const char *video_file = "/home/cona/Downloads/blais.mp4", *cover_file = "/home/cona/Downloads/blais.jpg";
    double f = 1000, cx = 320, cy = 240;
    size_t min_inlier_num = 100;

    cv::Ptr<cv::FeatureDetector> fdetector = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> fmatcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    // Load the object image and extract features
    // 기준 이미지
    cv::Mat obj_image = cv::imread(cover_file);
    if (obj_image.empty()) return -1;

    std::vector<cv::KeyPoint> obj_keypoint;
    cv::Mat obj_descriptor;
    //keypoint 및 descriptor 생성
    fdetector->detectAndCompute(obj_image, cv::Mat(), obj_keypoint, obj_descriptor);
    if (obj_keypoint.empty() || obj_descriptor.empty()) return -1;
    fmatcher->add(obj_descriptor);

    // Open a video
    cv::VideoCapture video;
    if (!video.open(video_file)) return -1;

    // Prepare a box for simple AR
    std::vector<cv::Point3f> box_lower = { cv::Point3f(30, 145,   0), cv::Point3f(30, 200,   0), cv::Point3f(200, 200,   0), cv::Point3f(200, 145,   0) };
    std::vector<cv::Point3f> box_upper = { cv::Point3f(30, 145, -50), cv::Point3f(30, 200, -50), cv::Point3f(200, 200, -50), cv::Point3f(200, 145, -50) };

    // Run pose estimation
    cv::Mat K = (cv::Mat_<double>(3, 3) << f, 0, cx, 0, f, cy, 0, 0, 1);
    cv::Mat dist_coeff = cv::Mat::zeros(5, 1, CV_64F), rvec, tvec;
    while (true)
    {
        // Read an image from the video
        cv::Mat img;
        video >> img;
        if (img.empty()) break;

        // Extract features and match them to the object features
        std::vector<cv::KeyPoint> img_keypoint;
        cv::Mat img_descriptor;
        fdetector->detectAndCompute(img, cv::Mat(), img_keypoint, img_descriptor);
        if (img_keypoint.empty() || img_descriptor.empty()) continue;
        std::vector<cv::DMatch> match;
        fmatcher->match(img_descriptor, match);
        if (match.size() < min_inlier_num) continue;
        std::vector<cv::Point3f> obj_points;
        std::vector<cv::Point2f> obj_project, img_points;
        for (auto m = match.begin(); m < match.end(); m++)
        {
            obj_points.push_back(cv::Point3f(obj_keypoint[m->trainIdx].pt));
            obj_project.push_back(obj_keypoint[m->trainIdx].pt);
            img_points.push_back(img_keypoint[m->queryIdx].pt);
        }

        // Determine whether each matched feature is an inlier or not
        std::vector<int> inlier;
        cv::solvePnPRansac(obj_points, img_points, K, dist_coeff, rvec, tvec, false, 500, 2, 0.99, inlier);
        cv::Mat inlier_mask = cv::Mat::zeros(int(match.size()), 1, CV_8U);
        for (size_t i = 0; i < inlier.size(); i++) inlier_mask.at<uchar>(inlier[i]) = 1;
        cv::Mat image_result;
        cv::drawMatches(img, img_keypoint, obj_image, obj_keypoint, match, image_result, cv::Vec3b(0, 0, 255), cv::Vec3b(0, 127, 0), inlier_mask);

        // Check whether inliers are enough or not
        size_t inlier_num = inlier.size();
        if (inlier_num > min_inlier_num)
        {
            // Estimate camera pose with inliers
            std::vector<cv::Point3f> obj_inlier;
            std::vector<cv::Point2f> img_inlier;
            for (int idx = 0; idx < inlier_mask.rows; idx++)
            {
                if (inlier_mask.at<uchar>(idx))
                {
                    obj_inlier.push_back(obj_points[idx]);
                    img_inlier.push_back(img_points[idx]);
                }
            }
            cv::solvePnP(obj_inlier, img_inlier, K, dist_coeff, rvec, tvec);

            // Draw the box on the image
            cv::Mat line_lower, line_upper;
            cv::projectPoints(box_lower, rvec, tvec, K, dist_coeff, line_lower);
            cv::projectPoints(box_upper, rvec, tvec, K, dist_coeff, line_upper);
            line_lower.reshape(1).convertTo(line_lower, CV_32S); // Change 4 x 1 matrix (CV_64FC2) to 4 x 2 matrix (CV_32SC1)
            line_upper.reshape(1).convertTo(line_upper, CV_32S); //  because 'cv::polylines()' only accepts 'CV_32S' depth.
            cv::polylines(image_result, line_lower, true, cv::Vec3b(255, 0, 0), 2);
            for (int i = 0; i < line_lower.rows; i++)
                cv::line(image_result, cv::Point(line_lower.row(i)), cv::Point(line_upper.row(i)), cv::Vec3b(0, 255, 0), 2);
            cv::polylines(image_result, line_upper, true, cv::Vec3b(0, 0, 255), 2);
        }

        // Show the image and process the key event
        cv::String info = cv::format("Inliers: %d (%d%%), Focal Length: %.0f", inlier_num, 100 * inlier_num / match.size(), K.at<double>(0));
        cv::putText(image_result, info, cv::Point(5, 15), cv::FONT_HERSHEY_PLAIN, 1, cv::Vec3b(0, 255, 0));
        cv::imshow("Pose Estimation (Book)", image_result);
        int key = cv::waitKey(1);
        if (key == 32) key = cv::waitKey(); // Space
        if (key == 27) break;               // ESC
    }

    video.release();
    return 0;
}

int main(int argc, char** argv)
{
    Node first_node, second_node;
    /*------------------------------2번 노드 pose 초기화(input: world기준 카메라 위치)------------------------------*/

    pose_t first_pose, second_pose;
    second_pose.R = Calculator::rpy_to_REigen(5, 5, 0);
    second_pose.t << 0.1, 0.1, 0.1;
    first_pose.t << 0.1, 0.1, 0.1;
    first_node.setCameraPose(first_pose);
    second_node.setCameraPose(second_pose);

    std::cout << "-----------ground truth-----------" << std::endl;
    std::cout << first_node.left_cam.pose_aff.matrix.inv() << std::endl;
    std::cout << second_node.left_cam.pose_aff.matrix.inv() << std::endl;
    std::cout << "----------------------" << std::endl;
    
    // PoseEstimation_Ex1();

    /*------------------------------world기준 point cloud 초기화------------------------------*/

    int num_points = 5;
    std::vector<cv::Point3f> point_cloud_;
    cv::Mat point_cloud(1, num_points*num_points, CV_32FC3);
    float gap = 0.1;
    float x = -gap*2.0, y = -gap*2.0, z = 2.0;
    
    for(int i = 0; i < point_cloud.cols; i++)
    {
        point_cloud.at<cv::Point3f>(i).x = x;
        point_cloud.at<cv::Point3f>(i).y = y;
        point_cloud.at<cv::Point3f>(i).z = z;
        point_cloud_.push_back(point_cloud.at<cv::Point3f>(i));
        
        // std::cout << x << ", " << y << ", " << z << std::endl;

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

    first_node.left_cam.SetImage(point_cloud);
    first_node.right_cam.SetImage(point_cloud);
    second_node.left_cam.SetImage(point_cloud);
    second_node.right_cam.SetImage(point_cloud);
    cv::imshow("n1 left_cam.image",first_node.left_cam.image);
    cv::imshow("n1 right_cam.image",first_node.right_cam.image);
    cv::imshow("n2 left_cam.image",second_node.left_cam.image);
    cv::imshow("n2 right_cam.image",second_node.right_cam.image);
    cv::waitKey(0);
    // cv::destroyAllWindows();

    cv::Mat dist_coeff = cv::Mat::zeros(5, 1, CV_64F), rvec, tvec, rot_mat;
    std::vector<int> inlier;
    //world 기준 3d point, 2번 노드 왼쪽 카메라 pixel -> world 기준 2번 노드 왼쪽 카메라의 pose
    cv::solvePnPRansac(point_cloud_, second_node.left_cam.points_pixel, first_node.left_cam.K_mat, dist_coeff, rvec, tvec, false, 500, 2, 0.99, inlier);
    cv::Rodrigues(rvec, rot_mat);

    std::cout << "-----------Pose Estimation-----------" << std::endl;
    std::cout << rot_mat.inv() << std::endl;
    std::cout << -rot_mat.inv()*tvec << std::endl;
    std::cout << "----------------------" << std::endl;

    /*------------------------------3D viewer------------------------------*/
    
    cv::viz::Viz3d myWindow("Coordinate Frame");
    myWindow.showWidget("n1 Left_Cam", cv::viz::WCoordinateSystem(), first_node.left_cam.pose_aff.inv());
    myWindow.showWidget("n1 Right_Cam", cv::viz::WCoordinateSystem(), first_node.right_cam.pose_aff.inv());
    myWindow.showWidget("n2 Left_Cam", cv::viz::WCoordinateSystem(), second_node.left_cam.pose_aff.inv());
    myWindow.showWidget("n2 Right_Cam", cv::viz::WCoordinateSystem(), second_node.right_cam.pose_aff.inv());
    myWindow.setWindowSize(cv::Size(1280,960));

    cv::viz::WCloud cloud_widget(point_cloud, cv::viz::Color::green());
    myWindow.showWidget("point_cloud", cloud_widget);

    while(!myWindow.wasStopped())
    {
        myWindow.spinOnce(1, true);
    }

    return 0;
}