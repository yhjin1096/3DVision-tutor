#include <3DVision-tutor/common.hpp>

int main(int argc, char** argv)
{
    /*------------------------------point cloud 및 stereo camera pose 초기화------------------------------*/
    Camera left_cam, right_cam;
    right_cam.pose_eig.t << -right_cam.base_line, 0.0, 0.0;
    left_cam.pose_aff = Calculator::Eig_to_Aff(left_cam.pose_eig);
    right_cam.pose_aff = Calculator::Eig_to_Aff(right_cam.pose_eig);

    int num_points = 5;
    cv::Mat point_cloud(1, num_points*num_points, CV_32FC3);
    float gap = 0.1;
    float x = -gap*2.0, y = -gap*2.0, z = 2.0;
    
    std::cout << "-----------ground truth-----------" << std::endl;
    for(int i = 0; i < point_cloud.cols; i++)
    {
        point_cloud.at<cv::Point3f>(i).x = x;
        point_cloud.at<cv::Point3f>(i).y = y;
        point_cloud.at<cv::Point3f>(i).z = z;
        
        std::cout << x << ", " << y << ", " << z << std::endl;

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
    std::cout << "----------------------" << std::endl;

    /*------------------------------point cloud 기반으로 이미지 만들기------------------------------*/

    std::cout << "-----------Triangulation-----------" << std::endl;
    left_cam.SetImage(point_cloud);
    right_cam.SetImage(point_cloud);
    cv::imshow("left_cam.image",left_cam.image);
    cv::imshow("right_cam.image",right_cam.image);
    cv::waitKey(0);
    cv::destroyAllWindows();

    // Calculator::Triangulation(left_cam, right_cam);
    Calculator::Triangulation_v2(left_cam, right_cam);
    std::cout << "----------------------" << std::endl;
    /*------------------------------3D viewer------------------------------*/
    
    cv::viz::Viz3d myWindow("Coordinate Frame");
    myWindow.showWidget("Left_Cam", cv::viz::WCoordinateSystem());
    myWindow.showWidget("Right_Cam", cv::viz::WCoordinateSystem(), right_cam.pose_aff.inv());
    myWindow.setWindowSize(cv::Size(1280,960));

    cv::viz::WCloud cloud_widget(point_cloud, cv::viz::Color::green());
    myWindow.showWidget("point_cloud", cloud_widget);

    while(!myWindow.wasStopped())
    {
        myWindow.spinOnce(1, true);
    }
    
    return 0;
}