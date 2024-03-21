#include "Stella_VPS/utils.hpp"
#include "Stella_VPS/mono_vo_extractor.h"

#include "Stella_VPS/orb_extractor.h"
#include "Stella_VPS/orb_extractor_node.h"
#include "Stella_VPS/orb_impl.h"
#include "Stella_VPS/orb_params.h"
#include <pangolin/pangolin.h>

int main(int argc, char** argv)
{
    FrameStereo refer, query;
    std::string path = "/home/cona/Downloads/dataset/data_odometry_gray/dataset/sequences/00/";
    int observe_gap = 3; // observe_gap = 3, refer image의 idx가 1이면, query idx는 2, 3, 4

    int num_images = 10;
    CountImages(num_images, path + "image_0/");
    GTPose gt_pose;
    gt_pose.readGTPose("/home/cona/Downloads/dataset/data_odometry_gray/data_odometry_poses/dataset/poses/00.txt");
    
    orb_params orb_parm("default ORB feature extraction setting");
    orb_extractor orb_etr(&orb_parm, 800);

    auto bf_matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
    std::vector<cv::DMatch> bf_matches;

    //projection matrix == K[R|t]
    cv::Mat P0 = (cv::Mat_<float>(3, 4) << 7.188560000000e+02, 0.000000000000e+00, 6.071928000000e+02, 0.000000000000e+00,
                                           0.000000000000e+00, 7.188560000000e+02, 1.852157000000e+02, 0.000000000000e+00,
                                           0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 0.000000000000e+00);
    cv::Mat P1 = (cv::Mat_<float>(3, 4) << 7.188560000000e+02, 0.000000000000e+00, 6.071928000000e+02, -3.861448000000e+02,
                                           0.000000000000e+00, 7.188560000000e+02, 1.852157000000e+02, 0.000000000000e+00,
                                           0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 0.000000000000e+00);

    pangolin::CreateWindowAndBind("Point cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1280, 720, 500, 500, 512, 389, 0.0001, 1000),
    pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));

    auto handler = std::make_unique<pangolin::Handler3D>(s_cam);
    pangolin::View &d_cam = pangolin::CreateDisplay()
                                .SetBounds(0.0, 1.0, 0.0, 1.0, -1280.0f / 720.0f)
                                .SetHandler(handler.get());

    for(int i = 0; i < num_images-observe_gap && !pangolin::ShouldQuit(); i++)
    {
        FrameStereo refer;
        refer.left_cam.image = cv::imread(path + "image_0/" + cv::format("%06d.png", i));
        refer.right_cam.image = cv::imread(path + "image_1/" + cv::format("%06d.png", i));
        
        cv::cvtColor(refer.left_cam.image, refer.left_cam.image, CV_BGR2GRAY);
        cv::cvtColor(refer.right_cam.image, refer.right_cam.image, CV_BGR2GRAY);
        // featureDetection(refer.left_cam.image, refer.left_cam.keypoints, refer.left_cam.points, refer.left_cam.descriptor);
        // featureDetection(refer.right_cam.image, refer.right_cam.keypoints, refer.right_cam.points, refer.right_cam.descriptor);
        orb_etr.extract(refer.left_cam.image, cv::Mat{}, refer.left_cam.keypoints, refer.left_cam.descriptor);
        orb_etr.extract(refer.right_cam.image, cv::Mat{}, refer.right_cam.keypoints, refer.right_cam.descriptor);
        // cv::KeyPoint::convert(refer.left_cam.keypoints, refer.left_cam.points, std::vector<int>());
        // cv::KeyPoint::convert(refer.right_cam.keypoints, refer.right_cam.points, std::vector<int>());
        
        bf_matcher->match(refer.left_cam.descriptor, refer.right_cam.descriptor, bf_matches);
        for (const auto &match : bf_matches)
            if (match.distance < 40)
                refer.stereo_matches.push_back(match);

        for(int k = 0; k < refer.stereo_matches.size(); k++)
        {
            refer.left_cam.points.push_back(cv::Point2f(refer.left_cam.keypoints[refer.stereo_matches[k].queryIdx].pt));
            refer.right_cam.points.push_back(cv::Point2f(refer.right_cam.keypoints[refer.stereo_matches[k].trainIdx].pt));
        }
        cv::Mat pts_4d;
        cv::triangulatePoints(P0, P1, refer.left_cam.points, refer.right_cam.points, pts_4d);
        
        /*-------------------------------------------------------*/
        //visualize stereo matching
        cv::Mat img_bf;
        cv::drawMatches(refer.left_cam.image, refer.left_cam.keypoints,
                        refer.right_cam.image, refer.right_cam.keypoints, refer.stereo_matches,
                        img_bf, cv::Scalar::all(-1), cv::Scalar::all(-1),
                        std::vector<char>(),
                        cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        cv::imshow("img_bf", img_bf);
        cv::waitKey(30);
        
        // visualize 3d
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glPointSize(3);

        // Draw World coordinate frame
        glBegin(GL_LINES);
        glColor3f(1.0, 0.0, 0.0);
        glVertex3d(0.0, 0.0, 0.0);
        glVertex3d(0.5, 0.0, 0.0);
        glColor3f(0.0, 1.0, 0.0);
        glVertex3d(0.0, 0.0, 0.0);
        glVertex3d(0.0, 0.5, 0.0);
        glColor3f(0.0, 0.0, 1.0);
        glVertex3d(0.0, 0.0, 0.0);
        glVertex3d(0.0, 0.0, 0.5);
        glEnd();

        glBegin(GL_POINTS);
        glColor3f(1.0, 0.0, 0.0);
        for (int i = 0; i < pts_4d.cols; i++) {
            cv::Mat x = pts_4d.col(i);
            x /= x.at<float>(3, 0);
            glVertex3d(x.at<float>(0, 0), x.at<float>(1, 0), x.at<float>(2, 0));
            // std::cout << x.at<float>(0, 0) << " " << x.at<float>(1, 0) << " " << x.at<float>(2, 0) << std::endl;
        }
        glEnd();


        pangolin::FinishFrame();
        usleep(5000); // sleep 5 ms
        /*-------------------------------------------------------*/

        // for(int j = 1; j <= observe_gap; j++)
        // {
        //     FrameStereo query;
        //     query.left_cam.image = cv::imread(path + "image_0/" + cv::format("%06d.png", i + j));
        //     query.right_cam.image = cv::imread(path + "image_1/" + cv::format("%06d.png", i + j));
            
        //     cv::cvtColor(query.left_cam.image, query.left_cam.image, CV_BGR2GRAY);
        //     cv::cvtColor(query.right_cam.image, query.right_cam.image, CV_BGR2GRAY);
        //     // orb_etr.extract(query.left_cam.image, cv::Mat{}, query.left_cam.keypoints, query.left_cam.descriptor);
        //     // orb_etr.extract(query.right_cam.image, cv::Mat{}, query.right_cam.keypoints, query.right_cam.descriptor);
            
        //     std::vector<uchar> status;
        //     std::vector<float> err;
        //     cv::TermCriteria termcrit = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);
        //     cv::calcOpticalFlowPyrLK(refer.left_cam.image, query.left_cam.image,
        //                             refer.left_cam.points, query.left_cam.points,
        //                             status, err, cv::Size(21, 21), 3, termcrit, 0, 0.001);
        // }
    }

    return 0;
}