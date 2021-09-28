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

#ifndef _VISUALIZER_H_
#define _VISUALIZER_H_

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include "opencv2/opencv.hpp"
#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <highgui.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "parameter.h"
#include "visualizer/DepthMapUtil.h"

class Visualizer
{
    public:
        Visualizer(camodocal::CameraPtr cam);
        ~Visualizer();
        void pubProjection(const Eigen::Matrix4d &transformation, const pcl::PointCloud<pcl::PointXYZ> &cloud, 
                    const cv::Mat &frame, pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud,double t);
        void pubTrackImg(const cv::Mat &imgTrack, double t);
        void pubNewCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud, double t);
        void pubOriginalOdom(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, double t);
        void pubNewOdom(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, double t);
        void pubCamNewOdom(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, double t);
        void pubExtrinsic(const Eigen::Vector3d &P, const Eigen::Matrix3d &R, double t);
        void pubKeyframePoints(const sensor_msgs::PointCloud &point_cloud);

        void generateRGBPointCloud();
        cv::Point2f Point3DTo2D(const pcl::PointXYZ &pt);
        void setDepth(const cv::Point2i, const float val);

        DepthCompletion depth_map_util;
        camodocal::CameraPtr cam_;
        ros::NodeHandle nh_;
        ros::Publisher pub_original_odom_, pub_new_odom_, pub_new_camera_odom_;
        ros::Publisher pub_projection_, pub_depth_map_, pub_img_track_;
        ros::Publisher pub_rgb_cloud_;
        ros::Publisher pub_extrinsic_;
        ros::Publisher pub_keyframe_point_;
};

#endif

