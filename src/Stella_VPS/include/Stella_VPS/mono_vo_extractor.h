//https://github.com/avisingh599/mono-vo.git
#include "Stella_VPS/utils.hpp"

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

void featureDetection(const cv::Mat &img, std::vector<cv::KeyPoint>& keypoints, std::vector<cv::Point2f>& points, cv::Mat& descriptor)
{
    // uses FAST as of now, modify parameters as necessary
    int fast_threshold = 20;
    bool nonmaxSuppression = true;
    cv::FAST(img, keypoints, fast_threshold, nonmaxSuppression);
    
    int indexCorrection = 0;
    int size = keypoints.size();
    
    // //filtering
    // for(int i = 0; i < size; i++)
    // {
    //   if(keypoints[i - indexCorrection].response < 100)
    //   {
    //     keypoints.erase(keypoints.begin() + (i-indexCorrection));
    //     indexCorrection++;
    //   }
    // }
    
    // cv::KeyPoint::convert(keypoints, points, std::vector<int>());

    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    orb->compute(img, keypoints, descriptor);
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