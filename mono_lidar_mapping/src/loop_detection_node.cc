/*******************************************************
* Copyright (C) 2020, Intelligent Positioning and Navigation Lab, Hong Kong Polytechnic University
*
* This file is part of lmono.
* Licensed under the GNU General Public License v3.0;
* you may not use this file except in compliance with the License.

* If you use this code, please cite the respective publications as
* listed on the above websites.
* 
* Author: Bo Zhang (dreamskybobo@gmail.com)
* Date: 2021/03/09
*******************************************************/
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <thread>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Quaternion.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>

#include "loop_parameter.h"
#include "utils/TicToc.h"
#include "loop_detection/Loop_Detector.h"

std::queue<sensor_msgs::ImageConstPtr> image_buf;
std::queue<sensor_msgs::PointCloudConstPtr> point_buf;
std::queue<nav_msgs::Odometry::ConstPtr> pose_buf;

std::mutex buf_mutex_;
std::mutex img_buf_mutex_;
std::mutex point_buf_mutex_;
std::mutex odom_buf_mutex_;
std::mutex process_mutex_;

double last_skip_time = -1;
Eigen::Vector3d last_T = Eigen::Vector3d(0,0,0);
int frame_index = 0;
int sequence =0;

camodocal::CameraPtr m_camera;
std::string IMAGE_TOPIC_0;
std::string CAM0;
int DEBUG_IMAGE;
int ROW;
int COL;
int SKIP_CNT;
double SKIP_DIS;
double SKIP_TIME;
std::string BRIEF_PATTERN_FILE;
double LOOP_SEARCH_TIME;
int LOOP_SEARCH_GAP;
int MIN_PNP_LOOP_NUM;
int MIN_BRIEF_LOOP_NUM;

Eigen::Vector3d tlc;
Eigen::Matrix3d qlc;

LoopDetector loop_detector;

ros::Publisher pub_matched_points_;
ros::Publisher pub_pnp_matched_img_;
ros::Publisher pub_brief_matched_img_;
ros::Publisher pub_correct_odom_;

void imageHandler(const sensor_msgs::ImageConstPtr &image_msg)
{
    //printf("receiving img %f\n", image_msg->header.stamp.toSec());
    img_buf_mutex_.lock();
    image_buf.push(image_msg);
    img_buf_mutex_.unlock();
}
void odomHandler(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    //printf("receive new odom %f\n", pose_msg->header.stamp.toSec());
    odom_buf_mutex_.lock();
    pose_buf.push(pose_msg);
    

    // printf("pose t: %f, %f, %f  pose q: %f, %f, %f, %f \n",pose_msg->pose.pose.position.x,
    //                                                         pose_msg->pose.pose.position.y,
    //                                                         pose_msg->pose.pose.position.z,
    //                                                         pose_msg->pose.pose.orientation.w,
    //                                                         pose_msg->pose.pose.orientation.x,
    //                                                         pose_msg->pose.pose.orientation.y,
    //                                                         pose_msg->pose.pose.orientation.z);
    odom_buf_mutex_.unlock();
}
void pointmsgHandler(const sensor_msgs::PointCloudConstPtr &point_msg)
{
    //printf("receive feature points %f\n", point_msg->header.stamp.toSec());

    // for (unsigned int i = 0; i < point_msg->points.size(); i++)
    // {
    //     printf("%d, 3D point: %f, %f, %f 2D point %f, %f \n",i , point_msg->points[i].x, 
    //                                                  point_msg->points[i].y,
    //                                                  point_msg->points[i].z,
    //                                                  point_msg->channels[i].values[0],
    //                                                  point_msg->channels[i].values[1]);
    // }

    point_buf_mutex_.lock();
    point_buf.push(point_msg);
    point_buf_mutex_.unlock();

}

void extrinsicHandler(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    process_mutex_.lock();
    tlc = Eigen::Vector3d(pose_msg->pose.pose.position.x,
                            pose_msg->pose.pose.position.y,
                            pose_msg->pose.pose.position.z);

    qlc = Eigen::Quaterniond(pose_msg->pose.pose.orientation.w,
                            pose_msg->pose.pose.orientation.x,
                            pose_msg->pose.pose.orientation.y,
                            pose_msg->pose.pose.orientation.z).toRotationMatrix();

    process_mutex_.unlock();
}

void process()
{
    while(true)
    {
        sensor_msgs::ImageConstPtr image_msg = NULL;
        sensor_msgs::PointCloudConstPtr point_msg = NULL;
        nav_msgs::Odometry::ConstPtr pose_msg = NULL;
        buf_mutex_.lock();

        if(!image_buf.empty() && !point_buf.empty() && !pose_buf.empty())
        {
            //printf("syncronize...\n");
            //printf("point buf %f\n", point_buf.front()->header.stamp.toSec());
            if(image_buf.front()->header.stamp.toSec() > pose_buf.front()->header.stamp.toSec())
            {
                pose_buf.pop();
                //printf("throwing pose %f\n", pose_buf.front()->header.stamp.toSec());

            }else if(image_buf.front()->header.stamp.toSec() > point_buf.front()->header.stamp.toSec())
            {
                point_buf.pop();
                //printf("throwing points %f\n", point_buf.front()->header.stamp.toSec());

            }else if(image_buf.back()->header.stamp.toSec() >= pose_buf.front()->header.stamp.toSec() && point_buf.back()->header.stamp.toSec() >= pose_buf.front()->header.stamp.toSec())
            {
                pose_msg = pose_buf.front();
                pose_buf.pop();

                //printf("obtain pose %f\n", pose_msg->header.stamp.toSec());

                while(!pose_buf.empty())
                {
                    //printf("throwing pose %f\n", pose_buf.front()->header.stamp.toSec());
                    pose_buf.pop();
                }

                while(image_buf.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec())
                {
                    //printf("throwing img %f\n", image_buf.front()->header.stamp.toSec());
                    image_buf.pop();
                }

                image_msg = image_buf.front();
                //printf("obtain img %f\n", image_msg->header.stamp.toSec());
                image_buf.pop();
                //printf("point buf %f\n", point_buf.front()->header.stamp.toSec());


                while(point_buf.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec())
                {
                    printf("throwing points %f\n", point_buf.front()->header.stamp.toSec());
                    point_buf.pop();
                }
                point_msg = point_buf.front();
                point_buf.pop();
                printf("img time: %f , feature time: %f, pose time: %f\n", image_msg->header.stamp.toSec(), point_msg->header.stamp.toSec(), pose_msg->header.stamp.toSec());
               
            }
        }

        buf_mutex_.unlock();

        if(pose_msg != NULL)
        {
            if(pose_msg->header.stamp.toSec()-last_skip_time < SKIP_TIME)
            {
                continue;
            }
            printf("processing points\n");
            cv_bridge::CvImageConstPtr ptr;
            if (image_msg->encoding == "8UC1")
            {
                ROS_INFO_STREAM("mono8");
                sensor_msgs::Image img;
                img.header = image_msg->header;
                img.height = image_msg->height;
                img.width = image_msg->width;
                img.is_bigendian = image_msg->is_bigendian;
                img.step = image_msg->step;
                img.data = image_msg->data;
                img.encoding = "mono8";
                ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);

            }else
                //ROS_INFO_STREAM("image type " << img_msg->encoding);
                ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
            
            cv::Mat image = ptr->image.clone();

            last_skip_time = pose_msg->header.stamp.toSec();

            Eigen::Vector3d T(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
            Eigen::Quaterniond Q(pose_msg->pose.pose.orientation.w, pose_msg->pose.pose.orientation.x, 
                                                    pose_msg->pose.pose.orientation.y, pose_msg->pose.pose.orientation.z);

            Eigen::Matrix3d R = Q.toRotationMatrix();
            
            if((T-last_T).norm() > SKIP_DIS)
            {
                std::vector<cv::Point3f> point_3d;
                std::vector<cv::Point2f> point_2d_uv;
                std::vector<cv::Point2f> point_2d_normal;
                std::vector<int> point_id;


                for(size_t i = 0; i < point_msg->points.size(); i++)
                {
                    cv::Point3f p_3d;
                    p_3d.x = point_msg->points[i].x;
                    p_3d.y = point_msg->points[i].y;
                    p_3d.z = point_msg->points[i].z;

                    point_3d.push_back(p_3d);

                    cv::Point2f p_2d_uv, p_2d_normal;
                    int p_id;

                    p_2d_normal.x = point_msg->channels[i].values[0];
                    p_2d_normal.y = point_msg->channels[i].values[1];
                    p_2d_uv.x = point_msg->channels[i].values[2];
                    p_2d_uv.y = point_msg->channels[i].values[3];
                    p_id = (int)point_msg->channels[i].values[4];

                    point_2d_uv.push_back(p_2d_uv);
                    point_2d_normal.push_back(p_2d_normal);
                    point_id.push_back(p_id);

                    // printf("receive publish point id %d, 3D point: %f, %f, %f, 2D uv point: %f, %f, 2D normal point: %f, %f\n",
                    //      p_id, p_3d.x, p_3d.y, p_3d.z, p_2d_uv.x,p_2d_uv.y, p_2d_normal.x, p_2d_normal.y);
                }
                
                printf("buidling keyframe database\n");
                KeyFrame* keyframe = new KeyFrame(pose_msg->header.stamp.toSec(), frame_index, T, R, image, point_3d, point_2d_uv, point_2d_normal, point_id, sequence);
                process_mutex_.lock();
                frame_index ++;
                loop_detector.addKeyFrame(keyframe,1);
                sequence++;
                last_T = T;
                process_mutex_.unlock();
            }
        }

        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"loop_detection");
    
    ros::NodeHandle nh("~");
    
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    ROS_WARN("waiting for msg");

    {
        std::string config_file;
        nh.getParam("loop_config_file", config_file);

        std::cout << "load: " << config_file << std::endl;

        FILE *fh = fopen(config_file.c_str(),"r");
        if(fh == NULL){
            ROS_WARN("config_file dosen't exist; wrong config_file path");
            ROS_BREAK();
            return 0;          
        }
        fclose(fh);

        cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            std::cerr << "ERROR: Wrong path to settings" << std::endl;
        }

        fsSettings["image_height"] >> ROW;
        fsSettings["image_width"] >> COL;
        fsSettings["image0_topic"] >> IMAGE_TOPIC_0;
        std::cout << "image0 topic: " << IMAGE_TOPIC_0 << std::endl;
        fsSettings["cam0_calib"] >> CAM0;
        std::cout << "cam0 calib: " << CAM0 << std::endl;

        fsSettings["skip_dis"] >> SKIP_DIS;
        fsSettings["skip_cnt"] >> SKIP_CNT;
        fsSettings["debug_image"] >> DEBUG_IMAGE;
        fsSettings["loop_search_time"] >> LOOP_SEARCH_TIME;
        fsSettings["loop_search_gap"] >> LOOP_SEARCH_GAP;
        fsSettings["skip_time"] >> SKIP_TIME;
        fsSettings["min_pnp_loop_num"] >> MIN_PNP_LOOP_NUM;
        fsSettings["min_brief_loop_num"] >> MIN_BRIEF_LOOP_NUM;

        m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(CAM0);

        BRIEF_PATTERN_FILE = "/home/bo/MonoLidarMapping/src/mono_lidar_mapping/support_files/brief_pattern.yml";
    }
    

    std::string vocabulary_file = "/home/bo/MonoLidarMapping/src/mono_lidar_mapping/support_files/brief_k10L6.bin";
    loop_detector.loadVocabulary(vocabulary_file);

    ros::Subscriber sub_extrinsic_ = nh.subscribe("/fused/extrinsic",2000,extrinsicHandler);
    ros::Subscriber sub_feature_points_ = nh.subscribe("/fused/keyframe_point",2000,pointmsgHandler);
    ros::Subscriber sub_image_ = nh.subscribe(IMAGE_TOPIC_0,2000, imageHandler);
    ros::Subscriber sub_odom_ = nh.subscribe("/fused/new_odometry",2000, odomHandler);

    pub_matched_points_ =  nh.advertise<sensor_msgs::PointCloud>("matched_points",1000);
    pub_pnp_matched_img_ = nh.advertise<sensor_msgs::Image>("pnp_matched_img",1000);
    pub_brief_matched_img_ = nh.advertise<sensor_msgs::Image>("brief_matched_img", 1000);
    pub_correct_odom_ = nh.advertise<nav_msgs::Odometry>("correct_odom",1000);

    std::thread measurement_process{process};

    ros::Rate r(5);
    while (ros::ok()) {
      ros::spinOnce();
      r.sleep();
    }

    return 0;
}
