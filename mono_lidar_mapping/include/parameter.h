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
#ifndef _PARAMETER_H_
#define _PARAMETER_H_

#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <string.h>
#include <cstdlib>
#include <cstdio>
#include <math.h>
#include <algorithm>
#include <map>
#include <utility>
#include <cmath>
#include <cv.h>
#include <opencv/highgui.h>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <boost/foreach.hpp>
#include "opencv2/opencv.hpp"

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

//#define UNIT_SPHERE_ERROR

#define RED   "\x1B[31m"
#define YEL   "\x1B[33m"
#define WHT   "\x1B[37m"

extern Eigen::Vector3d tlc;
extern Eigen::Matrix3d qlc;

const double FOCAL_LENGTH =460;
const int WINDOW_SIZE  = 10;
const int SKIP_FIRST_CNT = 10;

extern double MIN_PARALLAX;
extern std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
extern std::string LASER_ODOM_TOPIC;
extern std::string POINTS_TOPIC;
extern std::string IMU_TOPIC;

extern std::vector<std::string> CAM_NAMES;
extern Eigen::Matrix4d CAM0_T_CAM1;
extern Eigen::Matrix4d IMU_TO_CAM0;
extern Eigen::Matrix4d LASER_TO_CAM0;

extern double DELAY_TIME;

extern int OPEN_VISO;
extern int STEREO;
extern double CAMERA_HEIGHT;
extern double CAMERA_PITCH;
extern double FEATURE_THRESHOLD;
extern double SOLVER_TIME;
extern int NUM_ITERATIONS;
extern int FINE_TIMES;
extern double MIN_DIST;
extern int USE_IMU;
extern double PRIOR_T;
extern double PRIOR_R;
extern int FEATURE_SIZE;
extern double F_THRESHOLD;
extern double F_DIS;
extern int TRACK_CNT;
extern double OUTLIER_T;
extern int REJECT_F;

extern int ODOM_IO;
//extrinsic estimate cam0
extern int ESTIMATE_IMU;
extern int ESTIMATE_LASER;

extern double LASER_W;
extern double FACTOR_WEIGHT;
extern Eigen::Vector3d G;
extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;
extern double G_NORM;
extern double FILTER_SIZE;
extern std::string KERNEL_TYPE;
extern std::string BLUR_TYPE;
extern int KERNEL_SIZE;
extern double INIT_DEPTH;

//loop
extern int SKIP_CNT;
extern double SKIP_DIS;
extern int DEBUG_IMAGE;
extern std::string BRIEF_PATTERN_FILE;
extern int ROW;
extern int COL;
extern double LOOP_SEARCH_TIME;
extern double SKIP_TIME;
extern int LOOP_SEARCH_GAP;
extern cv::Mat MASK;
extern int MIN_PNP_LOOP_NUM;
extern int MIN_BRIEF_LOOP_NUM;
extern std::string CAM0;
//elas parameters
//extern struct ElasParam ELAS_PARAM;

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

void readParameters(ros::NodeHandle &n);

#endif