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

Adapted from VINS-Mono
*******************************************************/
#ifndef _FEATURE_MANAGER_H_
#define _FEATURE_MANAGER_H_

#include <iostream>
#include <fstream>
#include <string.h>
#include <cstdlib>
#include <cstdio>
#include <math.h>
#include <algorithm>
#include <map>
#include <utility>
#include <mutex>
#include <ceres/ceres.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

#include <cv.h>
#include <opencv/highgui.h>
#include <boost/foreach.hpp>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <opencv2/opencv.hpp>


#include "initial/SFM.h"
#include "initial/Solve_5pts.h"
#include "parameter.h"
#include "factor/ReprojectionFactor.h"

class FeaturePerFrame
{
    public:
        FeaturePerFrame(const Eigen::Matrix<double,6,1> &_point, double cur_t)
        {
            pt.x() = _point(0);
            pt.y() = _point(1);

            uv.x() = _point(2);
            uv.y() = _point(3);

            velocity.x() = _point(4);
            velocity.y() = _point(5);
            cur_td = cur_t;
            stereo = false;
        }

        void rightFrame(const Eigen::Matrix<double,6,1> &_point)
        {
            right_pt.x() = _point(0);
            right_pt.y() = _point(1);

            right_uv.x() = _point(2);
            right_uv.y() = _point(3);

            right_velocity.x() = _point(4);
            right_velocity.y() = _point(5);

            stereo = true;
        }

        double cur_td;
        bool stereo;
        Eigen::Vector2d pt, right_pt;
        Eigen::Vector2d uv, right_uv;
        Eigen::Vector2d velocity, right_velocity;

};

class ImageFrame
{
    public:
        ImageFrame(){};
        ImageFrame( const std::map<int,std::vector<std::pair<int, Eigen::Matrix<double,6,1>>>>& _points,double t,const Eigen::Matrix4d &laser_P,const Eigen::Matrix4d camera_P = Eigen::Matrix4d::Identity()):t{t}
        {   
            // std::cout << "cam_P: \n" << camera_P.matrix() << std::endl;
            // std::cout << "laser_P: \n" << laser_P.matrix() << std::endl;
            points = _points;
            R = camera_P.block<3,3>(0,0);
            T = camera_P.block<3,1>(0,3);

            L0_R = laser_P.block<3,3>(0,0);
            L0_T = laser_P.block<3,1>(0,3);
        }

        std::map<int,std::vector<std::pair<int, Eigen::Matrix<double,6,1>>>> points;
        double t;
        int key_frame;
        //IntegrationBase *pre_integration;
        Eigen::Matrix3d R;
        Eigen::Vector3d T;
        Eigen::Matrix3d L0_R;
        Eigen::Vector3d L0_T;
};

class FeaturePerId
{
    public:
        virtual ~FeaturePerId();
        int feature_id;
        int start_frame;
        std::vector<FeaturePerFrame> feature_per_frame;
        bool state;
        Eigen::Vector3d estimated_3d;
        double estimated_depth;
        int used_num;
        int solve_flag; //keyframe
        FeaturePerId() = delete;
        FeaturePerId(int _feature_id, int _start_frame)
        : feature_id(_feature_id), start_frame(_start_frame),
        estimated_3d(Eigen::Vector3d(0,0,0)),used_num(0), solve_flag(0), state(false)
        {
            estimated_depth = INIT_DEPTH;
        }

        int endFrame();
};

class FeatureManager
{
    public:
        FeatureManager();
        ~FeatureManager();
        
        bool featureCheck(int frame_count, const std::map<int,std::vector<std::pair<int, Eigen::Matrix<double,6,1>>>> &image, double td);
        void triangulate(int frameCnt, Eigen::Matrix3d Rs[], Eigen::Vector3d Ps[], Eigen::Matrix4d tlc = Eigen::Matrix4d::Identity());
        void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1, Eigen::Vector2d &point1, Eigen::Vector2d &point2, Eigen::Vector3d &point_3d);
        void triangulateFrames(int frame0, Eigen::Matrix4d &Pose0, int frame1,Eigen::Matrix4d &Pose1);
        bool addFeatureCheckParallax(int frame_count, const std::map<int, vector<pair<int, Eigen::Matrix<double, 6, 1>>>> &image, double td);
        double computeParallax(const FeaturePerId &it_per_id, int frame_count);
        void featureRefinement(Eigen::Matrix4d Pose[], Eigen::Matrix4d);
        void setDepth(const Eigen::VectorXd &x);
        int getFeatureCount();
        void removeOutlier(std::set<int> &outlierIndex);
        void removeFailures();
        void clearDepth();
        void clearState();
        void removeBack();
        void removeFront(int frame_count);
        void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P);
        bool relativePose(Eigen::Matrix3d &relative_R, Eigen::Vector3d &relative_T, int &l);
        bool initialStructure(std::vector<std::pair<double, ImageFrame>> &all_image_frame);

        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);
        Eigen::VectorXd getDepthVector();

        //StereoModel stereo_model;
        MotionEstimator m_estimator;
        std::list<FeaturePerId> feature;
        int new_feature_num;
        int last_track_num;
        int long_track_num;
        double para_depth_inv[10000][1];

        std::mutex m;

};

#endif
