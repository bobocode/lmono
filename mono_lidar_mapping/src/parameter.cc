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

//int WINDOW_SIZE;
double MIN_PARALLAX;
std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
std::string LASER_ODOM_TOPIC;
std::string POINTS_TOPIC;
std::string IMU_TOPIC;

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
double FACTOR_WEIGHT;
double PRIOR_T;
double PRIOR_R;
double FILTER_SIZE;
std::string KERNEL_TYPE;
std::string BLUR_TYPE;
int KERNEL_SIZE;

int ODOM_IO;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

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
    
    std::cout << "image0 topic: " << IMAGE0_TOPIC << std::endl;
    std::cout << "image1 topic: " << IMAGE1_TOPIC << std::endl;
    
    std::string cam0Calib;
    fsSettings["cam0_calib"] >> cam0Calib;

    std::cout << "cam0 calib: " << cam0Calib << std::endl;

    std::string cam1Calib;
    fsSettings["cam1_calib"] >> cam1Calib;

    std::cout << "cam1 calib: " << cam1Calib << std::endl;

    CAM_NAMES.push_back(cam0Calib);
    CAM_NAMES.push_back(cam1Calib);

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
    G.z() = G_NORM;
    //fsSettings["window_size"] >> WINDOW_SIZE;

    cv::Mat cv_T;
    fsSettings["cam0_T_cam1"] >> cv_T;
    cv::cv2eigen(cv_T, CAM0_T_CAM1);

    std::cout << "cam0 to cam1: \n" << CAM0_T_CAM1.matrix() << std::endl;

    if(USE_IMU)
    {
        cv::Mat imu_to_cam0;
        fsSettings["imu_T_cam0"] >> imu_to_cam0;
        cv::cv2eigen(imu_to_cam0, IMU_TO_CAM0);

        std::cout << "imu to cam0: \n" << IMU_TO_CAM0.matrix() << std::endl;
    }

}