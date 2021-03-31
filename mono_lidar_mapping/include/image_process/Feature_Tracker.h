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
#ifndef _FEATURE_TRACKER_H_
#define _FEATURE_TRACKER_H_

#include <cstdio>
#include <iostream>
#include <fstream>
#include <queue>
#include <mutex>
#include <execinfo.h>
#include <csignal>
#include <opencv2/opencv.hpp>
#include "cv.h"
#include <highgui.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <eigen3/Eigen/Dense>
//#include <viso_stereo.h>
#include <matrix.h>

#include "../parameter.h"

using namespace std;
using namespace Eigen;

class FeatureTracker
{
    public:
        FeatureTracker();
        ~FeatureTracker();
        map<int, vector<pair<int, Eigen::Matrix<double, 6, 1>>>> trackImage(double _cur_time, const cv::Mat &image_l, const cv::Mat &image_r = cv::Mat());     
        void setMask();
        vector<cv::Point2f> undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam);
        vector<cv::Point2f> ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                    map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts);
        void rejectWithF();
        void drawTrack(const cv::Mat &imLeft, cv::Mat &imRight,
                                   vector<int> &curLeftIds,
                                   vector<cv::Point2f> &curLeftPts, 
                                   map<int, cv::Point2f> &prevLeftPtsMap);
        
        double distance(cv::Point2f &pt1, cv::Point2f &pt2);

        void removeOutliers(set<int> &removePtsIds);
        cv::Mat getTrackImage();
        cv::Mat getDisparityImage();

        bool inBorder(const cv::Point2f &pt);
        void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
        void reduceVector(vector<int> &v, vector<uchar> status);

        int row, col;
        cv::Mat imTrack;
        cv::Mat imDisparity;

        cv::Mat mask;
        cv::Mat fisheye_mask; 
        cv::Mat prev_img, cur_img;
        vector<cv::Point2f> n_pts;
        vector<cv::Point2f> predict_pts;
        vector<cv::Point2f> predict_pts_debug;
        vector<cv::Point2f> prev_pts, cur_pts, cur_right_pts;
        vector<cv::Point2f> prev_un_pts, cur_un_pts, cur_un_right_pts;
        vector<cv::Point2f> prev_un_right_pts;
        vector<cv::Point2f> pts_velocity, right_pts_velocity;
        vector<int> ids, ids_right;
        vector<int> track_cnt;
        map<int, cv::Point2f> cur_un_pts_map, prev_un_pts_map;
        map<int, cv::Point2f> cur_un_right_pts_map, prev_un_right_pts_map;
        map<int, cv::Point2f> prevLeftPtsMap;

        int MAX_CNT;
        double prev_time;
        double cur_time;
        bool stereo_cam;
        int n_id;
        bool hasPrediction;

        int inputFrameCnt;
        int frame_count;

        std::vector<camodocal::CameraPtr> cams;
};

#endif