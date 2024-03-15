#include "Stella_VPS/utils.hpp"
#include "Stella_VPS/mono_vo_extractor.h"

#include "Stella_VPS/orb_extractor.h"
#include "Stella_VPS/orb_extractor_node.h"
#include "Stella_VPS/orb_impl.h"
#include "Stella_VPS/orb_params.h"

int main(int argc, char** argv)
{
    FrameMono refer, query;
    std::string path = "/home/cona/Downloads/dataset/data_odometry_gray/dataset/sequences/00/image_0/";
    int observe_gap = 3; // observe_gap = 3, refer image의 idx가 1이면, query idx는 2, 3, 4

    int num_images = 10;
    CountImages(num_images, path);
    GTPose gt_pose;
    gt_pose.readGTPose("/home/cona/Downloads/dataset/data_odometry_gray/data_odometry_poses/dataset/poses/00.txt");

    for(int i = 0; i < num_images-observe_gap; i++)
    {
        refer.image = cv::imread(path + cv::format("%06d.png", i));
        cv::cvtColor(refer.image, refer.image, CV_BGR2GRAY);
        for(int j = 1; j <= observe_gap; j++)
        {
            query.image = cv::imread(path + cv::format("%06d.png", i + j));
            cv::cvtColor(query.image, query.image, CV_BGR2GRAY);

            /*-------------------------*/
            /*----------first----------*/
            /*-------------------------*/
            // // feature extract: mono_vo (FAST)
            // // correspondence: mono_vo (KLT)
            // scale은 ground truth를 가져오는 것이므로 생략
            // double scale = 1.0;
            // scale = getAbsoluteScale(i+j, 0, t.at<double>(2));

            // std::vector<uchar> status; //tracking 성공 유무 저장
            // cv::Mat gt_rR, gt_rt;//gt relative rotation, translation
            // cv::Mat E, R, t, mask;
            // double rot_error, tr_error;
            // featureDetection(refer.image, refer.keypoints, refer.points, refer.descriptor);
            // featureTracking(refer.image, query.image, refer.points, query.points, status);
            // E = cv::findEssentialMat(query.points, refer.points, refer.focal, refer.pp, cv::RANSAC, 0.999, 1.0, mask);
            // cv::recoverPose(E, query.points, refer.points, R, t, refer.focal, refer.pp, mask);
            
            // cv::Mat test_image = refer.image.clone();
            // cv::cvtColor(test_image, test_image, CV_GRAY2BGR);
            // for(int k = 0; k < refer.points.size(); k++)
            //     cv::circle(test_image, refer.points[k], 2, CV_RGB(255,0,0));
            // cv::imshow("extract: mono, track: mono", test_image);
            // cv::waitKey(0);

            // gt의 relative pose 계산, error 계산
            // gt_rR = gt_pose.rotations[i].inv() * gt_pose.rotations[i+j];
            // gt_rt = gt_pose.translations[i+j] - gt_pose.translations[i];
            // rot_error = calculateRotationError(gt_rR, R);
            // tr_error = calculateTranslationError(gt_rt, t);

            // std::cout << "frame" << i << "(ref), " << "frame" << i+j  << "(query)" << std::endl;
            // std::cout << "rot_error(degree): " << rot_error << std::endl;
            // std::cout << "tr_error(m): " << tr_error << std::endl;

            /*-------------------------*/
            /*----------second---------*/
            /*-------------------------*/
            // // feature extract: mono_vo (FAST)
            // // correspondence: opencv ORB descriptor, KNN Match, BF Match
            // featureDetection(refer.image, refer.keypoints, refer.points, refer.descriptor);
            // featureDetection(query.image, query.keypoints, query.points, query.descriptor);
            
            // // match
            // auto bf_matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
            // auto knn_matcher = cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
            // std::vector<cv::DMatch> bf_matches;
            // std::vector<std::vector<cv::DMatch>> knn_matches;

            // bf_matcher->match(refer.descriptor, query.descriptor, bf_matches);
            // std::vector<cv::DMatch> good_bf_matches;
            // for (const auto &match : bf_matches)
            // {
            //     if (match.distance < 50)
            //     {
            //         good_bf_matches.push_back(match);
            //     }
            // }

            // knn_matcher.knnMatch(refer.descriptor, query.descriptor, knn_matches, 2);
            // constexpr auto ratio_thresh = 0.8;
            // std::vector<cv::DMatch> good_knn_matches;
            // for (const auto &match : knn_matches)
            // {
            //     if (match.size() != 0 && match[0].distance < ratio_thresh * match[1].distance)
            //     {
            //         good_knn_matches.push_back(match[0]);
            //     }
            // }
            // // 상위 top_n개의 match 사용
            // int top_n = 100;
            // std::sort(good_bf_matches.begin(), good_bf_matches.end());
            // std::sort(good_knn_matches.begin(), good_knn_matches.end());
            // std::vector<cv::DMatch> last_bf_match(good_bf_matches.begin(), good_bf_matches.begin()+top_n);
            // std::vector<cv::DMatch> last_knn_match(good_knn_matches.begin(), good_knn_matches.begin()+top_n);

            // std::vector<cv::Point2f> bf_refer_pts, bf_query_pts, knn_refer_pts, knn_query_pts;
            // for(int k = 0; k < last_bf_match.size(); k++)
            // {
            //     bf_refer_pts.push_back(refer.points[last_bf_match[k].queryIdx]);
            //     bf_query_pts.push_back(query.points[last_bf_match[k].trainIdx]);
            // }
            // for(int k = 0; k < last_knn_match.size(); k++)
            // {
            //     knn_refer_pts.push_back(refer.points[last_knn_match[k].queryIdx]);
            //     knn_query_pts.push_back(query.points[last_knn_match[k].trainIdx]);
            // }

            // //pose 계산
            // cv::Mat gt_rR, gt_rt;
            // cv::Mat bf_E, bf_R, bf_t, bf_mask, knn_E, knn_R, knn_t, knn_mask;

            // bf_E = cv::findEssentialMat(bf_query_pts, bf_refer_pts, refer.focal, refer.pp, cv::RANSAC, 0.999, 1.0, bf_mask);
            // cv::recoverPose(bf_E, bf_query_pts, bf_refer_pts, bf_R, bf_t, refer.focal, refer.pp, bf_mask);
            // knn_E = cv::findEssentialMat(knn_query_pts, knn_refer_pts, refer.focal, refer.pp, cv::RANSAC, 0.999, 1.0, knn_mask);
            // cv::recoverPose(knn_E, knn_query_pts, knn_refer_pts, knn_R, knn_t, refer.focal, refer.pp, knn_mask);

            // //visualize matches
            // cv::Mat img_bf;
            // cv::drawMatches(refer.image, refer.keypoints, query.image, query.keypoints, last_bf_match,
            //                 img_bf, cv::Scalar::all(-1), cv::Scalar::all(-1),
            //                 std::vector<char>(),
            //                 cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

            // cv::Mat img_knn;
            // cv::drawMatches(refer.image, refer.keypoints, query.image, query.keypoints,
            //                 last_knn_match, img_knn, cv::Scalar::all(-1),
            //                 cv::Scalar::all(-1), std::vector<char>(),
            //                 cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

            // cv::imshow("BF Matches", img_bf);
            // cv::imshow("KNN Matches", img_knn);
            // cv::waitKey(0);

            // // gt의 relative pose 계산, error 계산
            // double bf_rot_error, bf_tr_error, knn_rot_error, knn_tr_error;
            // gt_rR = gt_pose.rotations[i].inv() * gt_pose.rotations[i+j];
            // gt_rt = gt_pose.translations[i+j] - gt_pose.translations[i];
            // bf_rot_error = calculateRotationError(gt_rR, bf_R);
            // bf_tr_error = calculateTranslationError(gt_rt, bf_t);
            // knn_rot_error = calculateRotationError(gt_rR, knn_R);
            // knn_tr_error = calculateTranslationError(gt_rt, knn_t);

            // std::cout << "frame" << i << "(ref), " << "frame" << i+j  << "(query)" << std::endl;
            // std::cout << "bf_rot_error(degree): " << bf_rot_error << "  knn_rot_error(degree): " << knn_rot_error << std::endl;
            // std::cout << "bf_tr_error(m): " << bf_tr_error << "        knn_tr_error(m): " << knn_tr_error << std::endl;
            // std::cout << "" << std::endl;

            /*-------------------------*/
            /*----------third----------*/
            /*-------------------------*/
            orb_params orb_parm("default ORB feature extraction setting");
            orb_extractor orb_etr(&orb_parm, 800);

            orb_etr.extract(refer.image, cv::Mat{}, refer.keypoints, refer.descriptor);
            orb_etr.extract(query.image, cv::Mat{}, query.keypoints, query.descriptor);
            cv::KeyPoint::convert(refer.keypoints, refer.points, std::vector<int>());
            cv::KeyPoint::convert(query.keypoints, query.points, std::vector<int>());

            // match
            auto bf_matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
            auto knn_matcher = cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
            std::vector<cv::DMatch> bf_matches;
            std::vector<std::vector<cv::DMatch>> knn_matches;

            bf_matcher->match(refer.descriptor, query.descriptor, bf_matches);
            std::vector<cv::DMatch> good_bf_matches;
            for (const auto &match : bf_matches)
            {
                if (match.distance < 50)
                {
                    good_bf_matches.push_back(match);
                }
            }

            knn_matcher.knnMatch(refer.descriptor, query.descriptor, knn_matches, 2);
            constexpr auto ratio_thresh = 0.8;
            std::vector<cv::DMatch> good_knn_matches;
            for (const auto &match : knn_matches)
            {
                if (match.size() != 0 && match[0].distance < ratio_thresh * match[1].distance)
                {
                    good_knn_matches.push_back(match[0]);
                }
            }
            
            std::vector<cv::Point2f> bf_refer_pts, bf_query_pts, knn_refer_pts, knn_query_pts;
            for(int k = 0; k < good_bf_matches.size(); k++)
            {
                bf_refer_pts.push_back(refer.points[good_bf_matches[k].queryIdx]);
                bf_query_pts.push_back(query.points[good_bf_matches[k].trainIdx]);
            }
            for(int k = 0; k < good_knn_matches.size(); k++)
            {
                knn_refer_pts.push_back(refer.points[good_knn_matches[k].queryIdx]);
                knn_query_pts.push_back(query.points[good_knn_matches[k].trainIdx]);
            }

            //pose 계산
            cv::Mat gt_rR, gt_rt;
            cv::Mat bf_E, bf_R, bf_t, bf_mask, knn_E, knn_R, knn_t, knn_mask;

            bf_E = cv::findEssentialMat(bf_query_pts, bf_refer_pts, refer.focal, refer.pp, cv::RANSAC, 0.999, 1.0, bf_mask);
            cv::recoverPose(bf_E, bf_query_pts, bf_refer_pts, bf_R, bf_t, refer.focal, refer.pp, bf_mask);
            knn_E = cv::findEssentialMat(knn_query_pts, knn_refer_pts, refer.focal, refer.pp, cv::RANSAC, 0.999, 1.0, knn_mask);
            cv::recoverPose(knn_E, knn_query_pts, knn_refer_pts, knn_R, knn_t, refer.focal, refer.pp, knn_mask);

            //visualize matches
            cv::Mat img_bf;
            cv::drawMatches(refer.image, refer.keypoints, query.image, query.keypoints, good_bf_matches,
                            img_bf, cv::Scalar::all(-1), cv::Scalar::all(-1),
                            std::vector<char>(),
                            cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

            cv::Mat img_knn;
            cv::drawMatches(refer.image, refer.keypoints, query.image, query.keypoints,
                            good_knn_matches, img_knn, cv::Scalar::all(-1),
                            cv::Scalar::all(-1), std::vector<char>(),
                            cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

            cv::imshow("BF Matches", img_bf);
            cv::imshow("KNN Matches", img_knn);
            cv::waitKey(0);

            // gt의 relative pose 계산, error 계산
            double bf_rot_error, bf_tr_error, knn_rot_error, knn_tr_error;
            gt_rR = gt_pose.rotations[i].inv() * gt_pose.rotations[i+j];
            gt_rt = gt_pose.translations[i+j] - gt_pose.translations[i];
            bf_rot_error = calculateRotationError(gt_rR, bf_R);
            bf_tr_error = calculateTranslationError(gt_rt, bf_t);
            knn_rot_error = calculateRotationError(gt_rR, knn_R);
            knn_tr_error = calculateTranslationError(gt_rt, knn_t);

            std::cout << "frame" << i << "(ref), " << "frame" << i+j  << "(query)" << std::endl;
            std::cout << "bf_rot_error(degree): " << bf_rot_error << "  knn_rot_error(degree): " << knn_rot_error << std::endl;
            std::cout << "bf_tr_error(m): " << bf_tr_error << "        knn_tr_error(m): " << knn_tr_error << std::endl;
            std::cout << "" << std::endl;
        }
    }



    return 0;
}