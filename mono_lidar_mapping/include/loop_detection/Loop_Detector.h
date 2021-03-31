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

Adapted from VINS-mono.
*******************************************************/
#ifndef _LOOP_DETECTOR_H_
#define _LOOP_DETECTOR_H_

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
#include <sensor_msgs/PointCloud.h>

#include <cv_bridge/cv_bridge.h>

#include "loop_detection/DBoW/DBoW2.h"
#include "loop_detection/DBoW/TemplatedDatabase.h"
#include "loop_detection/DBoW/TemplatedVocabulary.h"
#include "loop_detection/DVision/DVision.h"

#include "loop_parameter.h"
#include "loop_detection/KeyFrame.h"
#include "utils/TicToc.h"
#include "utils/math_utils.h"

using namespace DVision;
using namespace DBoW2;
using namespace Eigen;

class LoopDetector
{
    public:
        LoopDetector();
        ~LoopDetector();

        int addKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop);
        void loadKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop);
        void loadVocabulary(std::string voc_path);
        void updateKeyFrameLoop(int index, Eigen::Matrix<double, 8, 1 > &_loop_info);
        KeyFrame* getKeyFrame(int index);
        cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg);
        void registerPub(ros::NodeHandle &n);

        Vector3d t_drift;
        double yaw_drift;
        Matrix3d r_drift;
        // world frame( base sequence or first sequence)<----> cur sequence frame  
        Vector3d w_t_vio;
        Matrix3d w_r_vio;

    private:
        int detectLoop(KeyFrame* keyframe, int frame_index);
        void addKeyFrameIntoVoc(KeyFrame* keyframe);
        void updatePath();
        list<KeyFrame*> keyframelist;
        std::mutex m_keyframelist;
        std::mutex m_optimize_buf;
        std::mutex m_path;
        std::mutex m_drift;
        std::thread t_optimization;
        std::queue<int> optimize_buf;

        int global_index;
        int sequence_cnt;
        vector<bool> sequence_loop;
        map<int, cv::Mat> image_pool;
        int earliest_loop_index;
        double last_loop_time;
        int base_sequence;

        BriefDatabase db;
        BriefVocabulary* voc;
};

template <typename T>
T NormalizeAngle(const T& angle_degrees) {
  if (angle_degrees > T(180.0))
  	return angle_degrees - T(360.0);
  else if (angle_degrees < T(-180.0))
  	return angle_degrees + T(360.0);
  else
  	return angle_degrees;
};

class AngleLocalParameterization {
 public:

  template <typename T>
  bool operator()(const T* theta_radians, const T* delta_theta_radians,
                  T* theta_radians_plus_delta) const {
    *theta_radians_plus_delta =
        NormalizeAngle(*theta_radians + *delta_theta_radians);

    return true;
  }

  static ceres::LocalParameterization* Create() {
    return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization,
                                                     1, 1>);
  }
};

template <typename T> 
void YawPitchRollToRotationMatrix(const T yaw, const T pitch, const T roll, T R[9])
{

	T y = yaw / T(180.0) * T(M_PI);
	T p = pitch / T(180.0) * T(M_PI);
	T r = roll / T(180.0) * T(M_PI);


	R[0] = cos(y) * cos(p);
	R[1] = -sin(y) * cos(r) + cos(y) * sin(p) * sin(r);
	R[2] = sin(y) * sin(r) + cos(y) * sin(p) * cos(r);
	R[3] = sin(y) * cos(p);
	R[4] = cos(y) * cos(r) + sin(y) * sin(p) * sin(r);
	R[5] = -cos(y) * sin(r) + sin(y) * sin(p) * cos(r);
	R[6] = -sin(p);
	R[7] = cos(p) * sin(r);
	R[8] = cos(p) * cos(r);
};

template <typename T> 
void RotationMatrixTranspose(const T R[9], T inv_R[9])
{
	inv_R[0] = R[0];
	inv_R[1] = R[3];
	inv_R[2] = R[6];
	inv_R[3] = R[1];
	inv_R[4] = R[4];
	inv_R[5] = R[7];
	inv_R[6] = R[2];
	inv_R[7] = R[5];
	inv_R[8] = R[8];
};

template <typename T> 
void RotationMatrixRotatePoint(const T R[9], const T t[3], T r_t[3])
{
	r_t[0] = R[0] * t[0] + R[1] * t[1] + R[2] * t[2];
	r_t[1] = R[3] * t[0] + R[4] * t[1] + R[5] * t[2];
	r_t[2] = R[6] * t[0] + R[7] * t[1] + R[8] * t[2];
};




#endif
