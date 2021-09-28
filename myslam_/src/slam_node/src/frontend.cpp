//
//

#include <opencv2/opencv.hpp>
#include "myslam/feature.h"
#include "myslam/frontend.h"

namespace myslam {

Frontend::Frontend() {
    gftt_ =
    cv::GFTTDetector::create(200, 0.01, 20);
    num_features_init_ = 100;
    num_features_ = 200;
}

bool Frontend::AddFrame(myslam::Frame::Ptr frame) {
    current_frame_ = frame;

    switch (status_) {
        case FrontendStatus::INITING:
            StereoInit();
            break;
        case FrontendStatus::TRACKING:
            Track();
            break;
    }

    last_frame_ = current_frame_;
    return true;
}

bool Frontend::Track() {

    int num_track_last = TrackLastFrame();
    //tracking_inliers_ = EstimateCurrentPose();

    if (num_track_last > 80) {
        // tracking good
        status_ = FrontendStatus::TRACKING;
    } else {
        //如果追到的点少于阈值，就认为它和上一关键帧间已经发生明显的位姿变化，设为新关键帧
        InsertKeyframe();
    }
    return true;
}

bool Frontend::InsertKeyframe() {

    current_frame_->SetKeyFrame();

    LOG(INFO) << "Set frame " << current_frame_->id_ << " as keyframe "
              << current_frame_->keyframe_id_;

    DetectFeatures();  // detect new features
    return 0;
}


int myslam::Frontend::TrackLastFrame() {
    // use LK flow to estimate points in the image
    //可以把上一帧特征点的位置作为当前帧的初值
    std::vector<cv::Point2f> kps_last, kps_current;
    for (auto &kp : last_frame_->features_left_) {

            kps_last.push_back(kp->position_.pt);
            kps_current.push_back(kp->position_.pt);
        }
    }

    std::vector<uchar> status;
    cv::Mat error;
    void cv::calcOpticalFlowPyrLK(
        this->last_frame_->left_img_, this->current_frame_->left_img_, kps_last,
        kps_current, status, error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_pts = 0; //这个东西是用来装返回的追踪点数量，没有别的意义

    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            cv::KeyPoint kp(kps_current[i], 7);
            //构造特征点对象，把持有它的帧和位置传入构造函数
            Feature::Ptr feature(new Feature(current_frame_, kp));
            
            //把这些特征点传入当前帧对象中
            current_frame_->features_left_.push_back(feature);
            num_good_pts++;
        }
    }

    LOG(INFO) << "Find " << num_good_pts << " in the last image.";
    return num_good_pts;
}

//这个东西要处理一下，它是用来实现双目初始化的，看看它在干什么
//当变成单目时，对应需求是什么，应该怎样改，怎样写
//想法1:单纯的对第一帧提个特征点就可以了；
bool myslam::Frontend::StereoInit() {
    this->current_frame_->SetKeyFrame(); //第一帧设为关键帧,这里可能写的有问题
    int num_features_left = myslam::Frontend::DetectFeatures(); //提特征点

    //InsertKeyframe()；
    myslam::Frontend::status_ = myslam::FrontendStatus::TRACKING; //修改状态
    return true; 
}

int myslam::Frontend::DetectFeatures() {
    cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
    for (auto &feat : current_frame_->features_left_) {
        cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                      feat->position_.pt + cv::Point2f(10, 10), 0, CV_FILLED);
    }

    std::vector<cv::KeyPoint> keypoints;
    gftt_->detect(current_frame_->left_img_, keypoints, mask);
    int cnt_detected = 0;
    for (auto &kp : keypoints) {
        current_frame_->features_left_.push_back(
            Feature::Ptr(new Feature(current_frame_, kp)));
        cnt_detected++;
    }

    LOG(INFO) << "Detect " << cnt_detected << " new features";
    return cnt_detected;
}


}  // namespace myslam