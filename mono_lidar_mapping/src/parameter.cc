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
#include "parameter.h"

Eigen::Vector3d tlc;
Eigen::Matrix3d qlc;

//int WINDOW_SIZE;
double MIN_PARALLAX;
std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
std::string LASER_ODOM_TOPIC;
std::string POINTS_TOPIC;
std::string IMU_TOPIC;
double DELAY_TIME;
int FINE_TIMES;
int FEATURE_SIZE;
double F_THRESHOLD;
double F_DIS;
int TRACK_CNT;
double OUTLIER_T;
int REJECT_F;
std::vector<std::string> CAM_NAMES;
int OPEN_VISO;
int STEREO;
double CAMERA_HEIGHT;
double CAMERA_PITCH;
double FEATURE_THRESHOLD;
double SOLVER_TIME;
double MIN_DIST;
int NUM_ITERATIONS;
int USE_IMU;
int ESTIMATE_IMU;
int ESTIMATE_LASER;
double LASER_W;
double ACC_N, ACC_W;
double GYR_N, GYR_W;
double G_NORM;
Eigen::Vector3d G{0.0, 0.0, 9.8};
Eigen::Matrix4d CAM0_T_CAM1;
Eigen::Matrix4d IMU_TO_CAM0;
Eigen::Matrix4d LASER_TO_CAM0;
double FACTOR_WEIGHT;
double PRIOR_T;
double PRIOR_R;
double FILTER_SIZE;
std::string KERNEL_TYPE;
std::string BLUR_TYPE;
int KERNEL_SIZE;
int DEBUG_IMAGE;

int ODOM_IO;

int SKIP_CNT;
double SKIP_DIS;
double SKIP_TIME;
std::string BRIEF_PATTERN_FILE;
double LOOP_SEARCH_TIME;
int LOOP_SEARCH_GAP;
int ROW;
int COL;
int MIN_PNP_LOOP_NUM;
int MIN_BRIEF_LOOP_NUM;
double INIT_DEPTH;

void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    config_file = readParam<std::string>(n,"config_file");
    FILE *fh = fopen(config_file.c_str(),"r");
    if(fh == NULL){
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK();
        return;          
    }
    fclose(fh);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["image0_topic"] >> IMAGE0_TOPIC;
    fsSettings["image1_topic"] >> IMAGE1_TOPIC;
    fsSettings["laser_odometry_topic"] >> LASER_ODOM_TOPIC;
    fsSettings["laser_topic"] >> POINTS_TOPIC;
    fsSettings["imu_topic"] >> IMU_TOPIC;
    fsSettings["delay_time"] >> DELAY_TIME;
    
    std::cout << "image0 topic: " << IMAGE0_TOPIC << std::endl;
    std::cout << "image1 topic: " << IMAGE1_TOPIC << std::endl;
    
    std::string cam0Calib;
    fsSettings["cam0_calib"] >> cam0Calib;
    std::cout << "cam0 calib: " << cam0Calib << std::endl;

    std::string cam1Calib;
    fsSettings["cam1_calib"] >> cam1Calib;
    std::cout << "cam1 calib: " << cam1Calib << std::endl;

    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];

    CAM_NAMES.push_back(cam0Calib);
    CAM_NAMES.push_back(cam1Calib);

    fsSettings["feature_size"] >> FEATURE_SIZE;
    fsSettings["f_threshold"] >> F_THRESHOLD;
    fsSettings["f_dis"] >> F_DIS;
    fsSettings["fine_times"] >> FINE_TIMES;
    fsSettings["track_cnt"] >> TRACK_CNT;
    fsSettings["outlier_t"] >> OUTLIER_T;
    fsSettings["use_rejectF"] >> REJECT_F;
    fsSettings["open_viso"] >> OPEN_VISO;
    fsSettings["stereo"] >> STEREO;
    fsSettings["camera_height"] >>CAMERA_HEIGHT;
    fsSettings["camera_pitch"] >> CAMERA_PITCH;
    fsSettings["feature_threshold"] >> FEATURE_THRESHOLD;
    fsSettings["solver_time"] >> SOLVER_TIME;
    fsSettings["num_iteration"] >> NUM_ITERATIONS;
    fsSettings["min_dist"] >> MIN_DIST;
    fsSettings["use_imu"] >> USE_IMU;
    fsSettings["acc_n"] >> ACC_N;
    fsSettings["acc_w"] >> ACC_W;
    fsSettings["gyr_n"] >> GYR_N;
    fsSettings["gyr_w"] >> GYR_W;
    fsSettings["g_norm"] >> G_NORM;
    fsSettings["estimate_imu"] >> ESTIMATE_IMU;
    fsSettings["estimate_laser"] >> ESTIMATE_LASER;
    fsSettings["factor_weight"] >> FACTOR_WEIGHT;
    fsSettings["laser_w"] >> LASER_W;
    fsSettings["odom_io"] >> ODOM_IO;
    fsSettings["prior_t"] >> PRIOR_T;
    fsSettings["prior_r"] >> PRIOR_R;
    fsSettings["filter_size"] >> FILTER_SIZE;
    fsSettings["kernel_type"] >> KERNEL_TYPE;
    fsSettings["blur_type"] >> BLUR_TYPE;
    fsSettings["kernel_size"] >> KERNEL_SIZE;
    fsSettings["skip_dis"] >> SKIP_DIS;
    fsSettings["skip_cnt"] >> SKIP_CNT;
    fsSettings["debug_image"] >> DEBUG_IMAGE;
    fsSettings["loop_search_time"] >> LOOP_SEARCH_TIME;
    fsSettings["loop_search_gap"] >> LOOP_SEARCH_GAP;
    fsSettings["skip_time"] >> SKIP_TIME;
    fsSettings["min_pnp_loop_num"] >> MIN_PNP_LOOP_NUM;
    fsSettings["min_brief_loop_num"] >> MIN_BRIEF_LOOP_NUM;
    G.z() = G_NORM;
    //fsSettings["window_size"] >> WINDOW_SIZE;

    INIT_DEPTH = -1.0;

    cv::Mat cv_cam0_T_cam1;
    fsSettings["cam0_T_cam1"] >> cv_cam0_T_cam1;
    cv::cv2eigen(cv_cam0_T_cam1, CAM0_T_CAM1);

    if(ESTIMATE_LASER != 2)
    {
        cv::Mat cv_laser_T_cam0;
        fsSettings["laser_to_camera0"] >> cv_laser_T_cam0;
        cv::cv2eigen(cv_laser_T_cam0, LASER_TO_CAM0);

        std::cout << "laser to cam0: \n" << LASER_TO_CAM0.matrix() << std::endl;
    }
    
    BRIEF_PATTERN_FILE = "/home/bo/lmono/src/mono_lidar_mapping/support_files/brief_pattern.yml";

    std::cout << "cam0 to cam1: \n" << CAM0_T_CAM1.matrix() << std::endl;

    // for(int i=0; i<ROW;++i)
    // {
    //     for(int j =0; j < COL; ++j)
    //     {
    //         if(j <256 || j > COL-256)
    //         {
    //             MASK.at<uchar>(i,j) = 0;
    //         }
    //     }
    // }
                                
    if(USE_IMU)
    {
        cv::Mat imu_to_cam0;
        fsSettings["imu_T_cam0"] >> imu_to_cam0;
        cv::cv2eigen(imu_to_cam0, IMU_TO_CAM0);

        std::cout << "imu to cam0: \n" << IMU_TO_CAM0.matrix() << std::endl;
    }

}