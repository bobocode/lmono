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
#ifndef _MAP_BUILDER_H_
#define _MAP_BUILDER_H_

#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <string>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <queue>
#include <assert.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <stdio.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <cv.h>
#include <highgui.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <cv_bridge/cv_bridge.h>
#include "mapping_parameter.h"
#include "visualizer/Visualizer.h"

class MapBuilder
{
    public:
        MapBuilder(){};
        ~MapBuilder(){};

        void processMapping();
        void associateToMap(const Eigen::Matrix4d &transformation, const pcl::PointCloud<pcl::PointXYZ> &cloud, 
                        cv::Mat &frame,const double t);

        void depthMap(const Eigen::Matrix4d &transformation, const pcl::PointCloud<pcl::PointXYZ> &cloud, 
                        cv::Mat &frame,const double t);

        void depthFill(cv::Mat &DMap);
        void generateRGBPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &rgb_cloud);
        cv::Point2f Point3DTo2D(const pcl::PointXYZ &pt);

        std::mutex map_mutex_;
        std::queue<pcl::PointCloud<pcl::PointXYZRGB>> rgb_points_buf;

};

#endif