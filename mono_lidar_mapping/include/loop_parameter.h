
#ifndef _LOOP_PARAMETER_H_
#define _LOOP_PARAMETER_H_

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

#define RED   "\x1B[31m"
#define YEL   "\x1B[33m"
#define WHT   "\x1B[37m"

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
extern int MIN_ORB_LOOP_NUM;
extern int USE_ORB;
extern double ANGLE_THRESHOLD;
extern double TRANS_THRESHOLD;
extern double SKIP_LOOP_TIME;
extern double SKIP_LOOP_DIS;
extern std::string CAM0;
extern Eigen::Vector3d tlc;
extern Eigen::Matrix3d qlc;
extern camodocal::CameraPtr m_camera;
extern std::string IMAGE_TOPIC_0;
extern cv::Mat MASK;
extern int IMAGE_CROP;

extern ros::Publisher pub_correct_odom_;
extern ros::Publisher pub_matched_points_;
extern ros::Publisher pub_pnp_matched_img_;
extern ros::Publisher pub_brief_matched_img_;
//elas parameters
//extern struct ElasParam ELAS_PARAM;

#endif