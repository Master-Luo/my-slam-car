//
// Created by gaoxiang on 19-5-2.
//

#ifndef MYSLAM_BACKEND_H
#define MYSLAM_BACKEND_H

#include "myslam/common_include.h"
#include "myslam/frame.h"

namespace myslam {

/**
 * 后端
 * 有单独优化线程，在Map更新时启动优化
 * Map更新由前端触发
 */ 

//上面是直接在c++框架下的逻辑，在ros下由于callback函数的存在，不需要考虑这个问题
class Backend {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Backend> Ptr;

    /// 构造函数中启动优化线程并挂起
    Backend();

    void myslam::Backend::feature_extract();

    void find_feature_matches();

    //pose_estimation
    void pose_estimation();
    

    /// 设置地图
    //void SetMap(std::shared_ptr<Map> map) { map_ = map; }

    /// 触发地图更新，启动优化
    //void UpdateMap();

    /// 关闭后端线程
   // void Stop();

   private:
    /// 后端线程
    //void BackendLoop();

    /// 对给定关键帧和路标点进行优化
    //void Optimize(Map::KeyframesType& keyframes, Map::LandmarksType& landmarks);

    //std::shared_ptr<Map> map_;
    //std::thread backend_thread_;
    //std::mutex data_mutex_;

    //std::condition_variable map_update_;
    //std::atomic<bool> backend_running_;

    //Camera::Ptr cam_left_ = nullptr, cam_right_ = nullptr;
};

}  // namespace myslam

#endif  // MYSLAM_BACKEND_H