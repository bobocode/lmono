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
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/io.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/registration/registration.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h> 
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <cv_bridge/cv_bridge.h>
#include "mapping_parameter.h"

class CloudVisualizer{
    public:
        CloudVisualizer();
        void updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
        void Spin();

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
        boost::mutex m;
        bool init;
};

class MapBuilder
{
    public:
        MapBuilder(){};
        ~MapBuilder(){};

        void processMapping();
        void associateToMap(const Eigen::Quaterniond &Q,const Eigen::Vector3d &T, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, 
                        cv::Mat &frame,const double t);

        void alignmentToMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud_ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud_ptr,pcl::PointCloud<pcl::PointXYZRGB>::Ptr transfor_cloud_ptr);

        void depthMap(const pcl::PointCloud<pcl::PointXYZ> &cloud, 
                        cv::Mat &frame,const double t);

        void depthFill(cv::Mat &DMap);
        void generateRGBPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &rgb_cloud);
        cv::Point2f Point3DTo2D(const pcl::PointXYZ &pt);

        std::mutex map_mutex_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_map;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr last_rgb_cloud;
        std::queue<std::pair<double, pcl::PointCloud<pcl::PointXYZRGB>>> rgb_points_buf;
        CloudVisualizer cloud_vis;

        int map_index =0;

};

#endif