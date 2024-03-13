// https://github.com/avisingh599/mono-vo.git
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <boost/filesystem.hpp>
#include <iostream>
#include <ctype.h>
#include <algorithm> // for copy
#include <iterator>  // for ostream_iterator
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>

const std::string imageExtensions[] = {".jpg", ".jpeg", ".png", ".gif", ".bmp"};

void featureTracking(const cv::Mat &prev_img, const cv::Mat &curr_img, std::vector<cv::Point2f> &prev_pts, std::vector<cv::Point2f> &curr_pts, std::vector<uchar> &status)
{
    // this function automatically gets rid of points for which tracking fails
    std::vector<float> err;
    cv::Size winSize = cv::Size(21, 21);
    cv::TermCriteria termcrit = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);

    cv::calcOpticalFlowPyrLK(prev_img, curr_img, prev_pts, curr_pts, status, err, winSize, 3, termcrit, 0, 0.001);

    // getting rid of points for which the KLT tracking failed or those who have gone outside the frame
    int indexCorrection = 0;
    for (int i = 0; i < status.size(); i++)
    {
        cv::Point2f pt = curr_pts.at(i - indexCorrection);
        
        if ((status.at(i) == 0) || (pt.x < 0) || (pt.y < 0))
        {
            if ((pt.x < 0) || (pt.y < 0))
            {
                status.at(i) = 0;
            }
            prev_pts.erase(prev_pts.begin() + (i - indexCorrection));
            curr_pts.erase(curr_pts.begin() + (i - indexCorrection));
            
            indexCorrection++;
        }
    }
}

void featureDetection(const cv::Mat &img, std::vector<cv::KeyPoint>& keypoints, std::vector<cv::Point2f>& points)
{
    // uses FAST as of now, modify parameters as necessary
    int fast_threshold = 20;
    bool nonmaxSuppression = true;
    cv::FAST(img, keypoints, fast_threshold, nonmaxSuppression);
    cv::KeyPoint::convert(keypoints, points, std::vector<int>());
}

inline void CountImages(int &num_images, const std::string &path)
{
    try
    {
        // 지정된 폴더 내의 모든 파일에 대해 반복
        for (const auto &entry : boost::filesystem::directory_iterator(path))
        {
            // 디렉토리인 경우 건너뛰기
            if (boost::filesystem::is_directory(entry.path()))
                continue;

            // 이미지 파일인 경우 개수 증가
            for (const std::string &ext : imageExtensions)
            {
                if (entry.path().extension() == ext)
                    num_images++;
            }
        }

        std::cout << "Number of image files in the folder: " << num_images << std::endl;
    }
    catch (const std::exception &ex)
    {
        std::cerr << "Error: " << ex.what() << std::endl;
    }
}

double getAbsoluteScale(int frame_id, int sequence_id, double z_cal)	{
  
  std::string line;
  int i = 0;
  std::ifstream myfile ("/home/cona/Downloads/dataset/data_odometry_gray/data_odometry_poses/dataset/poses/00.txt");
  double x =0, y=0, z = 0;
  double x_prev, y_prev, z_prev;
  if (myfile.is_open())
  {
    while (( getline (myfile,line) ) && (i<=frame_id))
    {
      z_prev = z;
      x_prev = x;
      y_prev = y;
      std::istringstream in(line);
      //cout << line << '\n';
      for (int j=0; j<12; j++)  {
        in >> z ;
        if (j==7) y=z;
        if (j==3)  x=z;
      }
      
      i++;
    }
    myfile.close();
  }

  else {
    std::cout << "Unable to open file";
    return 0;
  }

  return sqrt((x-x_prev)*(x-x_prev) + (y-y_prev)*(y-y_prev) + (z-z_prev)*(z-z_prev)) ;

}