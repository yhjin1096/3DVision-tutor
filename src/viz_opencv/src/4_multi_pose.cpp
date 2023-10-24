#include <iostream>
#include <opencv2/viz.hpp>
#include <opencv2/opencv.hpp>
#include <viz_opencv/common.hpp>

cv::Mat rpy_to_RMat(double x, double y, double z)
{
    Matrix3d Rx, Ry, Rz, result;
    
    x = x / 180.0 * CV_PI;
    y = y / 180.0 * CV_PI;
    z = z / 180.0 * CV_PI;

    // Rx << 1.0, 0.0, 0.0,
    //       0.0, cos(x), -sin(x),
    //       0.0, sin(x), cos(x);
    
    // Ry << cos(y), 0.0, sin(y),
    //       0.0, 1.0, 0.0,
    //       -sin(y), 0.0, cos(y);

    // Rz << cos(z), -sin(z), 0.0,
    //       sin(z), cos(z), 0.0,
    //       0.0, 0.0, 1.0;
          
    // result = Rz * Ry * Rx;

    
    result = Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX())
             * Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ());
    
    cv::Mat result_mat;
    result_mat = (cv::Mat_<float>(3,3) << result(0,0), result(0,1), result(0,2),
                                          result(1,0), result(1,1), result(1,2),
                                          result(2,0), result(2,1), result(2,2));
    
    return result_mat;
}

int main(int argc, char** argv)
{
    int num = 0;
    std::vector<int> node;
    
    cv::Mat rot_mat(3,3,CV_32FC1);
    rot_mat = (cv::Mat_<float>(3,3) << 1,0,0,0,1,0,0,0,1);
    float tr_increase = 0.01;
    cv::Affine3f pose;

    cv::viz::Viz3d myWindow("Coordinate Frame");
    myWindow.setWindowSize(cv::Size(1280,960));

    while(!myWindow.wasStopped())
    {   
        node.push_back(num);

        if(node.size() == 10)
            break;
        else
        {
            pose.rotation(rot_mat);
            pose.translation(cv::Vec3f(tr_increase,tr_increase,tr_increase));
            tr_increase+=0.15;
        }
        
        myWindow.showWidget(std::to_string(node[num]), cv::viz::WCoordinateSystem(), pose);            
        myWindow.spinOnce(500, true);
        num++;
    }

    return 0;
}