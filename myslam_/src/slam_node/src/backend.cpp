
//
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// #include "extra.h" // use this if in OpenCV2

#include  "myslam/frame.h"
#include "myslam/backend.h"
#include "myslam/feature.h"


namespace myslam {

//对图像提取特征点函数
void myslam::Backend::feature_extract(myslam::Frame::Ptr keyframe_)
{
  // used in OpenCV3
  cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
  cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
  // use this if you are in OpenCV2
  // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
  // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
  
  //-- 第一步:检测 Oriented FAST 角点位置
  detector->detect(keyframe_->left_img_, keyframe_->keypoints_);

  //-- 第二步:根据角点位置计算 BRIEF 描述子
  descriptor->compute(keyframe_->left_img_, keyframe_->keypoints_, keyframe_->descriptors_);
}


//两幅图像特征点匹配部分
void find_feature_matches(myslam::Frame::Ptr keyframe1_, myslam::Frame::Ptr keyframe2_,
std::vector<cv::DMatch> &matches) 
{
cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
  //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
std::vector<cv::DMatch> match;
  //BFMatcher matcher ( NORM_HAMMING );
matcher->match(keyframe1_->descriptors_, keyframe2_->descriptors_, match);

  //-- 第四步:匹配点对筛选
  double min_dist = 10000, max_dist = 0;

  //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
  for (int i = 0; i < keyframe1_->descriptors_.rows; i++) {
    double dist = match[i].distance;
    if (dist < min_dist) min_dist = dist;
    if (dist > max_dist) max_dist = dist;
  }

  //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
   for (int i = 0; i < keyframe1_->descriptors_.rows; i++) {
    if (match[i].distance <= std::max(2 * min_dist, 30.0)) {
      matches.push_back(match[i]);
    }
  }
}

//位姿计算部分
void pose_estimation_2d2d(std::vector<cv::KeyPoint> keypoints_1,
                          std::vector<cv::KeyPoint> keypoints_2,
                          std::vector<cv::DMatch> matches,
                          cv::Mat &R, cv::Mat &t) {
  // 相机内参,TUM Freiburg2
  cv::Mat K = (cv::Mat_<double>(3, 3) << 602.46, 0, 329.773, 0, 602.36, 238.683, 0, 0, 1);

  //-- 把匹配点转换为vector<Point2f>的形式
  std::vector<cv::Point2f> points1;
  std::vector<cv::Point2f> points2;

  for (int i = 0; i < (int) matches.size(); i++) {
    points1.push_back(keypoints_1[matches[i].queryIdx].pt);
    points2.push_back(keypoints_2[matches[i].trainIdx].pt);
  }

  //-- 计算基础矩阵
  cv::Mat fundamental_matrix;
  fundamental_matrix = cv::findFundamentalMat(points1, points2, CV_FM_8POINT);

  //-- 计算本质矩阵
  cv::Point2d principal_point(329.773, 238.683);  //相机光心
  double focal_length = 602;      //相机焦距, TUM dataset标定值
  cv::Mat essential_matrix;
  essential_matrix = cv::findEssentialMat(points1, points2, focal_length, principal_point);

  //-- 从本质矩阵中恢复旋转和平移信息.
  // 此函数仅在Opencv3中提供
  cv::recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
}


}  // namespace myslam