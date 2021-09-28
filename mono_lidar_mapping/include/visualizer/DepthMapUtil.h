
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
#ifndef _DEPTH_COMPLETION_H_
#define _DEPTH_COMPLETION_H_

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include "opencv2/opencv.hpp"
#include <cv.h>
#include <highgui.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include "parameter.h"

class DepthCompletion
{
    public:
        DepthCompletion();
        ~DepthCompletion();
        void depthCompletionFast(cv::Mat &depth_map);
        void depthCompletionMultiScale(cv::Mat &depth_map);
};

#endif
