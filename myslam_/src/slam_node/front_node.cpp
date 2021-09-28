/*  接受xx数据，发布xx话题，消息类型为xxx */
//写一个光流追踪，判断关键帧的前端

#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "frontend.h"


//现在是单目，先光流追踪特征点，再判断是否insertkeyframe()，不是则下一帧继续追踪
//判断依据为跟踪点数< 阈值，如小于100/50时，这个值好像mono的更小
//设置新的last_keyframe,curr_keyframe设为当前帧()

//得到带feature的图像和关键帧后，
//图像发布给rviz，关键帧指针话题给后端，让后端估计位姿和三角化


//先声明容器，为缓存图像和光流信息作准备

ros::Publisher pub_match;
ros::Publisher pub_keyframe; //

frontend_ = Frontend::Ptr(new Frontend);  


//定义img_callbackleft函数，它的输入是图像指针类的引用
void img_callbackleft(const sensor_msgs::ImageConstPtr &img_msg)
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
     //先初始化frame和前端,YAML文件配置系统参数
    
     Frame::Ptr new_frame = Frame::CreateFrame(); ;
     new_frame->left_img_ = ptr->image;
     if (new_frame == nullptr) return false;

     ///1.2.提取特征点 -》前端成员函数DetectFeatures()提取GFFT特征，封装到一个feature对象中
     //2.光流追踪 -》 调用LK光流，传入两帧图像和一帧特征点，得到当前帧特征点，封装到feature中
     //3.位姿估计 -》这部分无法调用，要写(自己写或别人写的)，pose放入frame中，//定位
     //4.判断是否为关键帧 -》bool is_keyframe_ ,由一个特征点数量阈值判断，如果不是关键帧，就回到2，
     //-》是关键帧，设置，2d-2d对极几何 求解两关键帧位姿变化，进入5
     //5.对新关键帧提取特征点，更新存放特征点的容器，再回到2
     //以上功能都封装再了frontend_->AddFrame函数中

     bool success = frontend_->AddFrame(new_frame);//会判断是否是关键帧

    if(frontend_->current_frame_->is_keyframe_ = true){
        pub_keyframe.publish(img_msg);
    }
 
     ///////////////////////////////////////////////////////////////////////////////////////////////////

     //0.系统初始化？？？怎么实现的，其中YAML文件配置的机制是什么，文件流读取参数
     //要干什么: 初始化/确定相机参数？？？/SFM
     //相机外参方法:YAML文件中写入默认参数，文件流读取，实现函数在parameter中

     //show_track是画带特征点的img的 
     //这里trackerData[i].cur_pts.size()是封装特征点的容器，要改
    if (SHOW_TRACK)
        {
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
            //cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
            cv::Mat origin_img = ptr->image;

            cv::Mat tmp_img = origin_img.rowRange( 0,  ROW);
            cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);

                //把点画到图上
                //for (auto &feat : current_frame_->features_left_)
                for (unsigned int i= 0; i< frontend_->current_frame_->features_left_.size(); i++)
                {
                    cv::circle(tmp_img, frontend_->current_frame_->features_left_[i], 2, cv::Scalar(0, 0, 255 ), 2);
            
                }
            }
            //cv::imshow("vis", stereo_img);
            //cv::waitKey(5);
            pub_match.publish(ptr->toImageMsg()); //mono里是这样写的，但是可能传不过去，看看结果
            //ptr转换的方式支持就地修改数据，如果没有成功，应该试一下把tmp_img改为ptr->img
        }
    ROS_INFO("whole feature tracker processing costs:");
}


//主函数应该是什么样子的
int main(int argc, char **argv)
{
    //ROS Initialize
    ros::init(argc, argv, "frontend_node");

    //create NodeHandle
    ros::NodeHandle n;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);


     //订阅左目话题IMAGE_TOPIC(/camera0/image_raw),有图像发布这个话题时，执行回调
     //正是由于回调函数这一特性，可以把函数封装进去，使他们自动循环执行
    ros::Subscriber sub_l=n.subscribe(IMAGE_TOPIC, 100, img_callbackleft);

    //3.上一步结束后，特征点和keyframe都得到了，发布它们
    //发布feature_img，传的应该是ptr指针
    ros::Publisher  pub_match = n.advertise<sensor_msgs::Image>("feature_img",1000);
    //发布关键帧，这里应该用什么类型，看看mono是用的什么消息类型
    ros::Publisher  pub_keyframe = n.advertise<sensor_msgs::Image>("keyframe_img",1000);
    
    //set loop
    ros::spin();
    return 0;
}





