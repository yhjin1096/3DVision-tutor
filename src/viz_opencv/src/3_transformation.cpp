#include <iostream>
#include <fstream>
#include <opencv2/viz.hpp>
#include <viz_opencv/common.hpp>

int main( int /*argc*/, char** /*argv*/ )
{
    //point cloud 파일 불러오기
    cv::Mat cloud(1, 1889, CV_32FC3);

    std::ifstream ifs("/home/cona/3DVision-tutor/src/viz_opencv/bunny.ply");
    
    std::string str;
    for(size_t i = 0; i < 12; ++i)
        std::getline(ifs, str);
    
    cv::Point3f* data = cloud.ptr<cv::Point3f>();
    float dummy1, dummy2;
    for(size_t i = 0; i < 1889; ++i)
        ifs >> data[i].x >> data[i].y >> data[i].z >> dummy1 >> dummy2;

    cloud *= 5.0f;
    /*----------------------------------------------------------------------*/
    //point cloud
    cv::viz::Viz3d myWindow("Coordinate Frame");
    myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
    myWindow.setWindowSize(cv::Size(1280,960));

    cv::viz::WCloud cloud_widget(cloud, cv::viz::Color::green());
    cv::Affine3f cloud_pose = cv::Affine3f().translate(cv::Vec3f(0.0f, 0.0f, 3.0f));
    
    myWindow.showWidget("bunny", cloud_widget, cloud_pose);

    /*----------------------------------------------------------------------*/
    //camera position 설정
    //cam_pos: global 기준 camera position, cam_focal_point: camera(z축)가 바라보는 global 좌표 기준 점, cam_y_dir: global 기준 카메라 y축 방향
    cv::Vec3f cam_pos(3.0f,3.0f,3.0f), cam_focal_point(3.0f,3.0f,4.0f), cam_y_dir(0.0f,1.0f,0.0f);
    //Affine3f -> R, t로 초기화 해도 됨
    cv::Affine3f cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
    //makeTransformToGlobal -> ?
    cv::Affine3f transform = cv::viz::makeTransformToGlobal(cv::Vec3f(0.0f,-1.0f,0.0f), cv::Vec3f(-1.0f,0.0f,0.0f), cv::Vec3f(0.0f,0.0f,-1.0f), cam_pos);

    // Coordinate axes
    cv::viz::WCameraPosition cpw(0.5);
    myWindow.showWidget("CPW", cpw, cam_pose);

    // Camera frustum, fov
    // cv::viz::WCameraPosition cpw_frustum(cv::Vec2f(0.889484, 0.523599));
    cv::Mat image = cv::imread("/home/cona/yhj/Screenshot from 2023-09-06 14-48-27.png");
    cv::viz::WCameraPosition cpw_frustum(cv::Vec2f(0.889484, 0.523599), image, 1.0, cv::Scalar(255, 255, 255));
    myWindow.showWidget("CPW_FRUSTUM", cpw_frustum, cam_pose);
    
    cv::Affine3f cloud_pose_global = transform * cloud_pose;
    myWindow.showWidget("bunny", cloud_widget, cloud_pose_global);

    while(!myWindow.wasStopped())
    {
        
        myWindow.spinOnce();
    }

    return 0;
}