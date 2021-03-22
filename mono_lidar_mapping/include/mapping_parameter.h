#ifndef _MAPPING_PARAMETER_H_
#define _MAPPING_PARAMETER_H_

#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fst
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

extern Eigen::Vector3d tlc;
extern Eigen::Matrix3d rlc;
extern std::string CAM0;
extern camodocal::CameraPtr m_camera;

extern ros::Publisher pub_depth_map_;
extern ros::Publisher pub_rgb_points_;
extern ros::Publisher pub_pro_img_;
extern int KERNEL_SIZE;

#endif