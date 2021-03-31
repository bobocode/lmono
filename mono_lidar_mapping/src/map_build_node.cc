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
#include "map_builder/Map_Builder.h"

Eigen::Vector3d tlc;
Eigen::Matrix3d rlc;
std::string CAM0;
camodocal::CameraPtr m_camera;
std::string IMAGE_TOPIC_0;
int KERNEL_SIZE;
double SKIP_DIS;
int FILTER_SIZE;
std::string KERNEL_TYPE;
std::string BLUR_TYPE;
double DELAY_TIME;
int SAVE_MAP;

std::queue<sensor_msgs::ImageConstPtr> image_buf;
std::queue<sensor_msgs::PointCloud2ConstPtr> point_buf;
std::queue<nav_msgs::Odometry::ConstPtr> pose_buf;

Eigen::Vector3d last_T = Eigen::Vector3d(0,0,0);

std::mutex buf_mutex_;
std::mutex img_buf_mutex_;
std::mutex point_buf_mutex_;
std::mutex odom_buf_mutex_;
std::mutex process_mutex_;

ros::Publisher pub_depth_map_;
ros::Publisher pub_rgb_points_;
ros::Publisher pub_pro_img_;
ros::Publisher pub_rgb_map_;

MapBuilder map_builder;

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
void pointsHandler(const sensor_msgs::PointCloud2ConstPtr &point_msg)
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
    while(true)
    {
        sensor_msgs::ImageConstPtr image_msg = NULL;
        sensor_msgs::PointCloud2ConstPtr point_msg = NULL;
        nav_msgs::Odometry::ConstPtr pose_msg = NULL;
        buf_mutex_.lock();

        if(!image_buf.empty() && !point_buf.empty() && !pose_buf.empty())
        {
            if(image_buf.front()->header.stamp.toSec() > pose_buf.front()->header.stamp.toSec())
            {
                pose_buf.pop();

            }else if(image_buf.front()->header.stamp.toSec() > point_buf.front()->header.stamp.toSec() + DELAY_TIME)
            {
                point_buf.pop();

            }else if(image_buf.front()->header.stamp.toSec() < point_buf.front()->header.stamp.toSec()-DELAY_TIME)
            {
                image_buf.pop();

            }else if(image_buf.back()->header.stamp.toSec() >= pose_buf.front()->header.stamp.toSec())
            {
                pose_msg = pose_buf.front();
                pose_buf.pop();

                while(!pose_buf.empty())
                {
                    //printf("throwing pose %f\n", pose_buf.front()->header.stamp.toSec());
                    pose_buf.pop();
                }

                while(image_buf.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec())
                {
                    //printf("throwing img %f\n", image_buf.front()->header.stamp.toSec());
                    image_buf.pop();
                    point_buf.pop();
                }

                image_msg = image_buf.front();
                //printf("obtain img %f\n", image_msg->header.stamp.toSec());
                image_buf.pop();

                point_msg = point_buf.front();
                point_buf.pop();

                //printf("img time: %f , cloud time: %f, pose time: %f\n", image_msg->header.stamp.toSec(), point_msg->header.stamp.toSec(), pose_msg->header.stamp.toSec());
            }
        }

        buf_mutex_.unlock();

        if(point_msg != NULL)
        {
            Eigen::Vector3d T(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
            Eigen::Quaterniond Q(pose_msg->pose.pose.orientation.w, pose_msg->pose.pose.orientation.x, 
                                                    pose_msg->pose.pose.orientation.y, pose_msg->pose.pose.orientation.z);
            if((T-last_T).norm() < SKIP_DIS)
            {
                continue;
            }

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
                ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

            cv::Mat image = ptr->image.clone();

            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*point_msg, *tmp_cloud);

            std::string map_file_path = "/home/bo/raw_data/map/" + std::to_string(point_msg->header.stamp.toNSec()) + ".pcd";
            pcl::io::savePLYFileBinary(map_file_path, *tmp_cloud);

            std::string img_file_path = "/home/bo/raw_data/map/" + std::to_string(point_msg->header.stamp.toNSec()) + ".jpg";
            cv::imwrite(img_file_path, image);

            Eigen::Matrix4d transformation;
            transformation.setIdentity();

            transformation.block<3,3>(0,0) = rlc.transpose();
            transformation.block<3,1>(0,3).setZero(); //= (-1) * rlc.transpose() * tlc;

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*tmp_cloud, *cloud_out, transformation);

            process_mutex_.lock();
            map_builder.associateToMap(Q, T, cloud_out,image, image_msg->header.stamp.toSec());
            process_mutex_.unlock();

        }

        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
        

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

        fsSettings["delay_time"] >> DELAY_TIME;
        fsSettings["kernel_type"] >> KERNEL_TYPE;
        fsSettings["blur_type"] >> BLUR_TYPE;
        fsSettings["kernel_size"] >> KERNEL_SIZE;
        fsSettings["skip_dis"] >> SKIP_DIS;
        fsSettings["filter_size"] >> FILTER_SIZE;
        fsSettings["image0_topic"] >> IMAGE_TOPIC_0;
        fsSettings["save_map"] >> SAVE_MAP;
    }

    ros::Subscriber sub_extrinsic_ = nh.subscribe("/fused/extrinsic",2000,extrinsicHandler);
    ros::Subscriber sub_point_cloud_ = nh.subscribe("/compact_data",2000,pointsHandler);
    ros::Subscriber sub_image_ = nh.subscribe(IMAGE_TOPIC_0,2000, imageHandler);
    ros::Subscriber sub_odom_ = nh.subscribe("/fused/new_camera_odometry",2000, odomHandler);

    pub_depth_map_ =  nh.advertise<sensor_msgs::Image>("depth_map",1000);
    pub_rgb_points_ = nh.advertise<sensor_msgs::PointCloud2>("rgb_points",1000);
    pub_pro_img_ = nh.advertise<sensor_msgs::Image>("pro_map", 1000);
    pub_rgb_map_ = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("rgb_map",5);

    std::thread measurement_process{process};
    std::thread map_manager(&MapBuilder::processMapping,&map_builder);
    //boost::thread visualizer(boost::bind(&CloudVisualizer::Spin, &(map_builder.cloud_vis)));

    ros::Rate r(5);
    while (ros::ok()) {
      ros::spinOnce();
      r.sleep();
    }

    return 0;


}