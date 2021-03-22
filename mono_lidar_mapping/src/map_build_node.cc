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
#include <sensor_msgs/PointCloud2.h>

#include "utils/TicToc.h"
#include "mapping_parameter.h"
#include "map_builder/map_builder.h"

Eigen::Vector3d tlc;
Eigen::Matrix3d rlc;
std::string CAM0;
camodocal::CameraPtr m_camera;
int KERNEL_SIZE;

std::queue<sensor_msgs::ImageConstPtr> image_buf;
std::queue<sensor_msgs::PointCloud2ConstPtr> point_buf;
std::queue<nav_msgs::Odometry::ConstPtr> pose_buf;

std::mutex buf_mutex_;
std::mutex img_buf_mutex_;
std::mutex point_buf_mutex_;
std::mutex odom_buf_mutex_;
std::mutex process_mutex_;

ros::Publisher pub_depth_map_;
ros::Publisher pub_rgb_points_;
ros::Publisher pub_pro_img_;

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
void pointmsgHandler(const sensor_msgs::PointCloud2ConstPtr &point_msg)
{
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

    rlc = Eigen::Quaterniond(pose_msg->pose.pose.orientation.w,
                            pose_msg->pose.pose.orientation.x,
                            pose_msg->pose.pose.orientation.y,
                            pose_msg->pose.pose.orientation.z).toRotationMatrix();

    process_mutex_.unlock();
}

void process()
{

}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"map_builder");
    
    ros::NodeHandle nh("~");
    
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    ROS_WARN("waiting for msg");

    ROS_WARN("waiting for msg");

    {
        std::string config_file;
        nh.getParam("map_config_file", config_file);

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

        fsSettings["cam0_calib"] >> CAM0;
        m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(CAM0);

    }
    ros::Subscriber sub_extrinsic_ = nh.subscribe("/fused/extrinsic",2000,extrinsicHandler);
    ros::Subscriber sub_point_cloud_ = nh.subscribe("/velodyne_points",2000,pointsHandler);
    ros::Subscriber sub_image_ = nh.subscribe("/image_left",2000, imageHandler);
    ros::Subscriber sub_odom_ = nh.subscribe("/fused/new_odometry",2000, odomHandler);

    pub_depth_map_ =  nh.advertise<sensor_msgs::PointCloud>("depth_map",1000);
    pub_rgb_points_ = nh.advertise<sensor_msgs::Image>("rgb_points",1000);
    pub_pro_img_ = nh.advertise<sensor_msgs::Image>("pro_map", 1000);

    std::thread measurement_process{process};

    ros::Rate r(5);
    while (ros::ok()) {
      ros::spinOnce();
      r.sleep();
    }

    return 0;


}