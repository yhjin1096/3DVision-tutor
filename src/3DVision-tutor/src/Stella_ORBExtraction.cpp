#include "3DVision-tutor/Stella_ORBExtraction.hpp"

const std::string imageExtensions[] = { ".jpg", ".jpeg", ".png", ".gif", ".bmp" };

inline void CountImages(int& num_images, const std::string& path)
{
    try {
        // 지정된 폴더 내의 모든 파일에 대해 반복
        for (const auto& entry : std::filesystem::directory_iterator(path)) {
            // 디렉토리인 경우 건너뛰기
            if (std::filesystem::is_directory(entry.path()))
                continue;

            // 이미지 파일인 경우 개수 증가
            for (const std::string& ext : imageExtensions) {
                if (entry.path().extension() == ext)
                    num_images++;
            }
        }

        std::cout << "Number of image files in the folder: " << num_images << std::endl;
    }
    catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
    }
}

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

        cv::Mat descriptors, out_descriptors;
        unsigned int num_keypts = 0;
        for (unsigned int level = 0; level < orb_params.num_levels_; ++level) {
            num_keypts += all_keypts.at(level).size();
        }
        if (num_keypts == 0) {
            out_descriptors.release();
        }
        else {
            out_descriptors.create(num_keypts, 32, CV_8U);
            descriptors = out_descriptors.clone();
        }
        unsigned int offset = 0;
        for(unsigned int level = 0; level < orb_params.num_levels_; level++)
        {
            auto& keypts_at_level = all_keypts.at(level);
            const auto num_keypts_at_level = keypts_at_level.size();
            if (num_keypts_at_level == 0) {
                continue;
            }

            cv::Mat blurred_image = orb_extractor.image_pyramid_.at(level).clone();
            cv::GaussianBlur(blurred_image, blurred_image, cv::Size(7, 7), 2, 2, cv::BORDER_REFLECT_101);

            cv::Mat descriptors_at_level = descriptors.rowRange(offset, offset + num_keypts_at_level);
            orb_extractor.compute_orb_descriptors(blurred_image, keypts_at_level, descriptors_at_level);

            offset += num_keypts_at_level;
            
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