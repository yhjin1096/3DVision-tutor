#include "Stella_VPS/utils.hpp"
#include "Stella_VPS/mono_vo_extractor.h"

int main(int argc, char** argv)
{
    Frame refer, query;
    std::string path = "/home/cona/Downloads/dataset/data_odometry_gray/dataset/sequences/00/image_0/";
    int observe_gap = 1; // observe_gap = 3, refer image의 idx가 1이면, query idx는 2, 3, 4

    int num_images = 10;
    CountImages(num_images, path);
    GTPose gt_pose;
    gt_pose.readGTPose("/home/cona/Downloads/dataset/data_odometry_gray/data_odometry_poses/dataset/poses/00.txt");

    for(int i = 0; i < num_images-observe_gap; i++)
    {
        refer.image = cv::imread(path + cv::format("%06d.png", i));
        cv::cvtColor(refer.image, refer.image, CV_BGR2GRAY);
        for(int j = 1; j <= observe_gap; j++)
        {
            query.image = cv::imread(path + cv::format("%06d.png", i + j));
            cv::cvtColor(query.image, query.image, CV_BGR2GRAY);

            //feature extract - mono_vo (FAST)
            //correspondence - mono_vo (KLT)
            std::vector<cv::KeyPoint> refer_keypts;
            std::vector<cv::Point2f> refer_points, query_points;
            std::vector<uchar> status;
            cv::Mat E, R, t, mask;
            double scale = 1.0;
            scale = getAbsoluteScale(i+j, 0, t.at<double>(2));
            featureDetection(refer.image, refer_keypts, refer_points);
            featureTracking(refer.image, query.image, refer_points, query_points, status);
            E = cv::findEssentialMat(query_points, refer_points, refer.focal, refer.pp, cv::RANSAC, 0.999, 1.0, mask);
            cv::recoverPose(E, query_points, refer_points, R, t, refer.focal, refer.pp, mask);
            std::cout << R << std::endl;
            std::cout << t << std::endl;
        }
    }



    return 0;
}