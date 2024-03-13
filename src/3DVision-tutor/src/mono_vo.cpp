//https://github.com/avisingh599/mono-vo.git
#include "3DVision-tutor/mono_vo.hpp"

int main(int argc, char** argv)
{
    cv::Mat left_image, right_image, prv_image;
    std::string left_path = "/home/cona/Downloads/dataset/data_odometry_gray/dataset/sequences/00/image_0/";
    std::string right_path = "/home/cona/Downloads/dataset/data_odometry_gray/dataset/sequences/00/image_1/";

    int num_images = 0;
    CountImages(num_images, left_path);
    
    std::vector<cv::KeyPoint> curr_keypoint, curr_keypoint_save, prev_keypoint;
    std::vector<cv::Point2f> curr_points, curr_points_save, prev_points;
    cv::Mat E, R, t, R_f, t_f, mask;
    double scale = 1.00;
    double focal = 718.8560;
    cv::Point2d pp(607.1928, 185.2157);

    //visualize
    cv::Mat traj = cv::Mat::zeros(600, 600, CV_8UC3);
    char text[100];
    int fontFace = cv::FONT_HERSHEY_PLAIN;
    double fontScale = 1;
    int thickness = 1;  
    cv::Point textOrg(10, 50);

    for(int i = 0; i < num_images; i++)
    {
        left_image = cv::imread(left_path + cv::format("%06d.png", i));

        if(left_image.empty())
        {
            std::cerr << "image not found" << std::endl;
            return -1;
        }
        if(left_image.type() != CV_8UC1)
            cv::cvtColor(left_image, left_image, CV_BGR2GRAY);

        curr_keypoint.clear();
        curr_points.clear();

        featureDetection(left_image, curr_keypoint, curr_points);
        curr_keypoint_save = curr_keypoint;
        curr_points_save = curr_points;

        if(i != 0)
        {
            std::vector<uchar> status;
            
            featureTracking(prv_image, left_image, prev_points, curr_points, status);
            E = cv::findEssentialMat(curr_points, prev_points, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
            cv::recoverPose(E, curr_points, prev_points, R, t, focal, pp, mask);
            
            if(i==1)
            {
                R_f = R.clone();
                t_f = t.clone();
            }
            
            cv::Mat prevPts(2, prev_points.size(), CV_64F), currPts(2, curr_points.size(), CV_64F);

            for(int j = 0; j < prev_points.size(); j++)
            {
                prevPts.at<double>(0,j) = prev_points.at(j).x;
                prevPts.at<double>(1,j) = prev_points.at(j).y;

                currPts.at<double>(0,j) = curr_points.at(j).x;
                currPts.at<double>(1,j) = curr_points.at(j).y;
            }

            scale = getAbsoluteScale(num_images, 0, t.at<double>(2));

            if ((scale>0.1)&&(t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1)))
            {
                t_f = t_f + scale*(R_f*t);
                R_f = R*R_f;
            }
        }
        
        prv_image = left_image.clone();

        prev_keypoint.clear();
        prev_points.clear();

        prev_keypoint = curr_keypoint_save;
        prev_points = curr_points_save;

        //visualize
        if(i > 1)
        {
            std::cout << t_f << std::endl;
            int x = int(t_f.at<double>(0)) + 300;
            int y = int(t_f.at<double>(2)) + 100;
            cv::circle(traj, cv::Point(x, y) ,1, CV_RGB(255,0,0), 2);

            cv::rectangle( traj, cv::Point(10, 30), cv::Point(550, 50), CV_RGB(0,0,0), CV_FILLED);
            sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
            cv::putText(traj, text, textOrg, fontFace, fontScale, cv::Scalar::all(255), thickness, 8);

            cv::imshow( "Road facing camera", left_image);
            cv::imshow( "Trajectory", traj );

            char key = cv::waitKey(1);
            if(key == 27)
                break;
        }
    }
    return 0;
}