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

#ifndef _ESTIMATOR_H_
#define _ESTIMATOR_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <fstream>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ceres/ceres.h>
//libviso
#include <viso_stereo.h>
#include <viso_mono.h>
#include <matrix.h>
//elas

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <Eigen/Eigen>

#include "initial/AxxbSolver.h"
#include "initial/SFM.h"
#include "initial/Solve_5pts.h"
#include "image_process/Camera_Model.h"
#include "image_process/Measurement_Manager.h"
#include "image_process/Feature_Manager.h"
#include "image_process/Feature_Tracker.h"
#include "factor/MarginalizationFactor.h"
#include "factor/MonoProjectionFactor.h"
#include "factor/PoseLocalParameterization.h"
#include "factor/LaserFactor.h"
#include "factor/PriorFactor.h"
#include "laser_odometry/PointMapping.h"
#include "visualizer/Visualizer.h"


using namespace lclio;

class Estimator: public MeasurementManager, public PointMapping
{
    public:
        Estimator();
        ~Estimator();

        void setParameter();
        void setupRos(ros::NodeHandle &nh);
        void inputImage(const double t, const cv::Mat &img0, const cv::Mat &img1 = cv::Mat());
        void processEstimation();
        void processLaserOdom(const lclio::Transform &transform_in, const std_msgs::Header &header);
        Eigen::Matrix4d processCompactData(const sensor_msgs::PointCloud2ConstPtr &compact_data, const std_msgs::Header &header);
        void processImage(const double &header,const cv::Mat &img0,
                Eigen::Matrix4d &transform_to_init, const cv::Mat &img1 = cv::Mat());
        cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg);
        bool visoHandler(const cv::Mat &img0, Eigen::Matrix4d &pos, const cv::Mat &img1 = cv::Mat());
        bool readIntrinsicYml(const std::string& filename);
        void resetViso();

// #ifdef USE_CORNER
//   void CalculateFeatures(const pcl::KdTreeFLANN<PointT>::Ptr &kdtree_surf_from_map,
//                          const PointCloudPtr &local_surf_points_filtered_ptr,
//                          const PointCloudPtr &surf_stack,
//                          const pcl::KdTreeFLANN<PointT>::Ptr &kdtree_corner_from_map,
//                          const PointCloudPtr &local_corner_points_filtered_ptr,
//                          const PointCloudPtr &corner_stack,
//                          const Transform &local_transform,
//                          vector<unique_ptr<Feature>> &features);

//   void CalculateLaserOdom(const pcl::KdTreeFLANN<PointT>::Ptr &kdtree_surf_from_map,
//                           const PointCloudPtr &local_surf_points_filtered_ptr,
//                           const PointCloudPtr &surf_stack,
//                           const pcl::KdTreeFLANN<PointT>::Ptr &kdtree_corner_from_map,
//                           const PointCloudPtr &local_corner_points_filtered_ptr,
//                           const PointCloudPtr &corner_stack,
//                           Transform &local_transform,
//                           vector<unique_ptr<Feature>> &features);
// #else
//   void CalculateFeatures(const pcl::KdTreeFLANN<PointT>::Ptr &kdtree_surf_from_map,
//                          const PointCloudPtr &local_surf_points_filtered_ptr,
//                          const PointCloudPtr &surf_stack,
//                          const Transform &local_transform,
//                          vector<unique_ptr<Feature>> &features);

//   void CalculateLaserOdom(const pcl::KdTreeFLANN<PointT>::Ptr &kdtree_surf_from_map,
//                           const PointCloudPtr &local_surf_points_filtered_ptr,
//                           const PointCloudPtr &surf_stack,
//                           Transform &local_transform,
//                           vector<unique_ptr<Feature>> &features);
// #endif
        bool optimization();
        void margin();
        void matrix2Double();
        void double2Matrix();
        void check();
        void slideWindow();
        void slideWindowOld();
        void slideWindowNew();
        void outliersRejection(std::set<int> &removeIndex,const double &error);
        double reprojectionError(Eigen::Matrix3d &Ri, Eigen::Vector3d &Pi, Eigen::Matrix3d &Rj,Eigen::Vector3d &Pj, Eigen::Vector2d &uvi, Eigen::Vector2d &uvj, double depth, Eigen::Matrix4d &EX_PARA);
        bool solveInitialEx(std::vector<std::pair<double, ImageFrame>> all_image_frame,Eigen::VectorXd &x);
        bool runInitialization();


        enum MarginalizationFlag
        {
            MARGIN_OLD = 0,
            MARGIN_SECOND_NEW = 1
        };

        enum EstimatorStageFlag {
            NOT_INITED,
            INITED,
        };

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

        int inputImageCnt;
        int frame_count;
        double cur_time;
        double prev_time;
        Eigen::Matrix4d TLC[2];

        std::vector<camera_calib> cameras;
        std::vector<std::pair<double, ImageFrame>> all_image_frame;
        std::queue<pair<double, std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 6, 1>>>>>> featureBuf;

        cv::Mat image_l,image_r;
        Eigen::Quaterniond L0_Q;
        Eigen::Vector3d L0_P;
        Eigen::Matrix4d L0_Pos;
        Eigen::Matrix4d C0_Pos;
        
        Eigen::Matrix4d prev_cam_pose;
        Eigen::Matrix4d prev_laser_pose;

        Eigen::Matrix4d prior_trans;
        
        //std::vector<camera_calib> cameras;

        FeatureTracker feature_tracker;
        FeatureManager feature_manager;
        AXXBSolver axxbsolver;

        CloudVisualizer cloud_vis;

    protected:

        Eigen::Matrix3d back_R0;
        Eigen::Vector3d back_P0;

        Eigen::Vector3d Ps[(WINDOW_SIZE+1)];
        Eigen::Matrix3d Rs[(WINDOW_SIZE+1)];

        double Header[(WINDOW_SIZE+1)];
        double para_pose[(WINDOW_SIZE+1)][7];
        double para_depth_inv[10000][1];
        double para_ex[1][7];
        double para_speed_bias[(WINDOW_SIZE+1)][9];

        lclio::Transform transform_tobe_mapped_bef_;
        lclio::Transform transform_es_;
        lclio::Transform transform_lc_{Eigen::Quaternionf(1, 0, 0, 0), Eigen::Vector3f(0, 0, -0.1)}; ///

        MarginalizationInfo *last_marginalization_info;
        std::vector<double *> last_marginalization_parameter_blocks;
        MarginalizationFlag marginalization_flag;

        EstimatorStageFlag stage_flag;

        int lost_frame_num;
        bool get_lost;
        bool change_reference_frame;
        boost::shared_ptr<viso::VisualOdometryMono> mono_visual_odometer_;
        viso::VisualOdometryMono::parameters mono_visual_odometer_params_;

        boost::shared_ptr<Visualizer> visualizer_;
        

};





#endif