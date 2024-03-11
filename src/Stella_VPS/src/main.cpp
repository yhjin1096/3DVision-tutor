#include "Stella_VPS/utils.hpp"
#include "Stella_VPS/orb_extractor.h"
#include "Stella_VPS/orb_extractor_node.h"
#include "Stella_VPS/orb_impl.h"
#include "Stella_VPS/orb_params.h"

enum mode
{
    SAVE,
    LOAD
};

int main(int argv, char **argc)
{
    int count = 0, num_images = 0;
    std::string left_path = "/home/cona/Downloads/dataset/data_odometry_gray/dataset/sequences/05/image_0/";
    std::string right_path = "/home/cona/Downloads/dataset/data_odometry_gray/dataset/sequences/05/image_1/";
    CountImages(num_images, left_path);
    cv::Mat left_image;

    mode mode = SAVE;

    if (mode == SAVE)
    {
        while (count < num_images)
        {
            left_image = cv::imread(left_path + cv::format("%06d.png", count));
            
            if(left_image.empty())
            {
                std::cerr << "image not found" << std::endl;
                return -1;
            }
            if(left_image.type() != CV_8UC1)
                cv::cvtColor(left_image, left_image, CV_BGR2GRAY);

            orb_params orb_parm("default ORB feature extraction setting");
            orb_extractor orb_etr(&orb_parm, 800);
            std::vector<cv::KeyPoint> keypoint;
            cv::Mat descriptor;
            orb_etr.extract(left_image, cv::Mat{}, keypoint, descriptor);
            


            count++;
        }
    }
    else if (mode == LOAD)
    {
    }

    return 0;
}