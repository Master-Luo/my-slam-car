/*  接受xx数据，发布xx话题，消息类型为xxx */
//写一个基于特征点匹配实现位姿估计和路标点计算的后端

#include <iostream>
#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include"myslam/backend.h"
#include"myslam/frame.h"
#include"myslam/frontend.h"

//这里pose估计要重写，原来是基于双目 三角化再pnp估计位姿

//订阅关键帧指针话题，估计last_keyframe和当前帧间的位姿变化，和三角化landmark
//回调函数中先对当前帧重新提取特征点，然后两个关键帧之间匹配，用对积几何，估算位姿
//发布pose和landmark的topic

 cv::Mat R;
 cv::Mat t;
 std::vector<cv::DMatch> matcher;

 ros::Publisher pub_pose;

 //frontend_ = Frontend::Ptr(new Frontend);  

 last_keyframe = Frame::Ptr( new Frame );
 current_keyframe = Frame::Ptr( new Frame );
 backend_ = Backend::Ptr(new Backend);
 
 //new Frame current_keyframe;

//keyframe callback function define,这里似乎可以重写，这里应该传入，定位到一个帧的容器
//这里大概明白为什么要有map这个容器来持有帧了，便于读取调用
//这里可以改一下，简化map为持有关键帧的一个滑动窗口容器，不占用过多内存
//如果有map，直接传keyframe_id就可以定位从中取出关键帧了

void kfcallback(const std_msgs::String::ConstPtr& msg,&last_keyframe, &current_keyframe)
{
     //1.获取图像，接收camera节点发布的topic -》通过封装到一个frame对象里实现接收，
    //图片类型由cv_bridge转换到opencv可读的类型
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

     //到此已经得到了一个opencv的可读图片，存放在一个ptr类对象中，就可以处理这张图像了

     current_keyfram->left_img_= ptr->image;

     //先对最新帧进行特征提取
     backend_-> feature_extract(current_keyfram);
    
     //先读取关键帧image给current_keyframe，如果是第一帧，不处理

     if (last_keyframe->left_img_=nullptr){
        last_keyframe =current_keyframe;
    }
    else{

        
     //先对两幅图像进行特征点匹配，orb，
        backend_->find_feature_matches(last_keyframe, current_keyframe, &matcher);
    
     //把上一帧和当前帧匹配的特征点共同传入位姿估计函数，R和t存放输出结果
        backend_->pose_estimation(last_keyframe->keypoints_,
                          current_keyframe->keypoints_ ,
                          matcher,
                          Mat &R, Mat &t);

     //三角化也可以考虑一下cv::trangulatePoints
     //当然，如果要三角化，可以先把两幅图像提取特征匹配的函数拆出来；
    
     //得到R，t ，封装成topic发布pose,只发布t

     visualization_msgs::Marker keyframe_pose;
    keyframe_pose.header = pose_msg->header;
    keyframe_pose.header.frame_id = "world";
    keyframe_pose.ns = "key_odometrys";
    keyframe_pose.type = visualization_msgs::Marker::SPHERE_LIST;
    keyframe_pose.action = visualization_msgs::Marker::ADD;
    keyframe_pose.pose.orientation.w = 1.0;
    keyframe_pose.lifetime = ros::Duration();

    //static int key_odometrys_id = 0;
    keyframe_pose.id = 0; //key_odometrys_id++;
    keyframe_pose.scale.x = 0.1;
    keyframe_pose.scale.y = 0.1;
    keyframe_pose.scale.z = 0.1;
    keyframe_pose.color.r = 1.0;
    keyframe_pose.color.a = 1.0;

    //
    geometry_msgs::Point pose_marker;
    //mat.at<uchar>(row,col)
    pose_marker.x = t.at<float>(0,0);
    pose_marker.y = t.at<float>(1,0);
    pose_marker.z = t.at<float>(2,0);
    keyframe_pose.points.push_back(pose_marker);

    pub_pose.publish(keyframe_pose);

    }

    last_keyframe = current_keyframe;
    current_keyframe = nullptr;
    matcher.clear();
    R.release;
    t.release;

}


int  main(int argc, char **argv)
{
    //ROS initialize
    ros::init(argc, argv, "backend" );

    //NodeHandle create
    ros::NodeHandle n;

    //create Subscriber and subscribe to the chatter topic and function chatterCallback
    ros::Subscriber sub=n.subscribe("keyframe_img",1000,kfcallback);

    //大概可以用pose，会最简单geometry_msgs::Pose
    ros::Publisher  pub_pose = n.advertise<visualization_msgs::Marker>("pose",1000);

    //wait
    ros::spin();

    return 0;
}


