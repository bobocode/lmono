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
#include <boost/foreach.hpp>
#include "opencv2/opencv.hpp"

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

//#define UNIT_SPHERE_ERROR

const double FOCAL_LENGTH =460;
const int WINDOW_SIZE  = 10;

extern double MIN_PARALLAX;
extern std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
extern std::string LASER_ODOM_TOPIC;
extern std::string POINTS_TOPIC;
extern std::string IMU_TOPIC;

extern std::vector<std::string> CAM_NAMES;
extern Eigen::Matrix4d CAM0_T_CAM1;
extern Eigen::Matrix4d IMU_TO_CAM0;

extern int OPEN_VISO;
extern int STEREO;
extern double CAMERA_HEIGHT;
extern double CAMERA_PITCH;
extern double FEATURE_THRESHOLD;
extern double SOLVER_TIME;
extern int NUM_ITERATIONS;
extern double MIN_DIST;
extern int USE_IMU;

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

//elas parameters
extern struct ElasParam ELAS_PARAM;

void readParameters(ros::NodeHandle &n);

#endif