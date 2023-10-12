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

// int main( int /*argc*/, char** /*argv*/ )
// {
//     cv::viz::Viz3d myWindow("Coordinate Frame");
//     cv::Mat image = cv::imread("/home/cona/yhj/Screenshot from 2023-09-06 14-48-27.png");
//     cv::Mat image2 = cv::imread("/home/cona/3d_vision/L_image.png");
//     myWindow.showImage(image, image.size());

//     while(!myWindow.wasStopped())
//     {
//         //3초 후 이미지 변경
//         myWindow.spinOnce(3000, true);
//         myWindow.showImage(image2, image2.size());
//     }
//     return 0;
// }

int main( int /*argc*/, char** /*argv*/ )
{
    cv::viz::Viz3d myWindow("Coordinate Frame");
    myWindow.setWindowSize(cv::Size(1280,960));

    //widget 목록 https://docs.opencv.org/3.4/d7/dcd/classcv_1_1viz_1_1Widget.html
    myWindow.showWidget("Coordinate", cv::viz::WCoordinateSystem());

    //Line
    cv::viz::WLine axis(cv::Point3f(-1.0f,-1.0f,-1.0f), cv::Point3f(1.0f,1.0f,1.0f));
    axis.setRenderingProperty(cv::viz::LINE_WIDTH, 4.0);
    myWindow.showWidget("Line Widget", axis);

    //cube
    cv::viz::WCube cube_widget(cv::Point3f(0.5,0.5,0.0), cv::Point3f(0.0,0.0,-0.5), true, cv::viz::Color::blue());
    cube_widget.setRenderingProperty(cv::viz::LINE_WIDTH, 4.0);
    myWindow.showWidget("Cube Widget", cube_widget);
    
    cv::Mat rot_vec = cv::Mat::zeros(1,3,CV_32F);
    float translation_phase = 0.0;
    float rot_increase = 1, tr_increase = 0.01;
    cv::Mat rot_mat;
    double r = 45, p = 0, y = 0;
    
    while(!myWindow.wasStopped())
    {
        r += rot_increase;
        p += rot_increase;
        y += rot_increase;
        rot_mat = rpy_to_RMat(r, p, y);

        translation_phase += (float)CV_PI * 0.01f;
        tr_increase = sin(translation_phase);
        
        cv::Affine3f pose(rot_mat, cv::Vec3f(tr_increase, tr_increase, tr_increase));

        myWindow.setWidgetPose("Cube Widget", pose);
        myWindow.spinOnce(1, true);
    }
    return 0;
}