#include "3DVision-tutor/Stella_ORBExtraction.hpp"


int main(int argc, char** argv)
{
    int num_images=0;
    std::string left_path = "/home/cona/Downloads/dataset/data_odometry_gray/dataset/sequences/05/image_0/";
    std::string right_path = "/home/cona/Downloads/dataset/data_odometry_gray/dataset/sequences/05/image_1/";
    cv::Mat left_image, right_image;

    orb_params orb_params("default ORB feature extraction setting");
    ORBExtractor orb_extractor(&orb_params);
    int cell_size = 8;
    
    CountImages(num_images, left_path);

    for(int i = 0; i < num_images; i++)
    {
        left_image = cv::imread(left_path + cv::format("%06d.png", i));
        right_image = cv::imread(right_path + cv::format("%06d.png", i));
        
        if(left_image.empty() || right_image.empty())
        {
            std::cerr << "image not found" << std::endl;
            return -1;
        }
        cv::Mat image = left_image.clone();
        cv::cvtColor(image, image, CV_8UC1);
        
        // orb_extractor.ExtractORB_cv(left_image, true);
        // orb_extractor.ExtractORB_cv(left_image, cell_size, true);

        std::vector<std::vector<cv::KeyPoint>> all_keypts;
        std::vector<cv::KeyPoint> keypts;
        orb_extractor.compute_image_pyramid(image);
        // for(int j = 0; j < orb_extractor.image_pyramid_.size(); j++)
        //     cv::imshow("image pyramid" + std::to_string(j), orb_extractor.image_pyramid_[j]);
        // cv::waitKey(0);

        orb_extractor.compute_fast_keypoints(all_keypts, cv::Mat());
        for(unsigned int level = 0; level < orb_params.num_levels_; level++)
        {
            auto& keypts_at_level = all_keypts.at(level);
            //descriptor 생성
            orb_extractor.correct_keypoint_scale(keypts_at_level, level);
            keypts.insert(keypts.end(), keypts_at_level.begin(), keypts_at_level.end());
        }
        // for(int j = 0; j < keypts.size(); j++)
        // {
        //     cv::circle(left_image, keypts[j].pt, 4, cv::Scalar(0,0,255),1);
        // }
        // cv::imshow("lef", left_image);
        // cv::waitKey(0);

        break;
    }
    
    return 0;
}