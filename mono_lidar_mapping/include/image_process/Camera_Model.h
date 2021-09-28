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
#ifndef _CAMERA_MODEL_H_
#define _CAMERA_MODEL_H_

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>
//#include <viso_stereo.h>

struct camera_calib
{
    std::string modelType;
    std::string camera_name;

    double fx;
    double fy;
    double cx;
    double cy;

    double Tx;
    double Ty;

    int image_width;
    int image_height;

    double k1, k2, p1, p2;
};

class StereoModel
{   
    public:
        StereoModel();
        ~StereoModel();

        const camera_calib left();
        const camera_calib right();
        const cv::Matx44d& reprojectionMatrix();
        bool fromCameraCalib(const camera_calib &left, const camera_calib &right);
        void projectDisparityTo3d(const cv::Point2d& left_uv_rect, float disparity, cv::Point3d& xyz) const;

        double baseline() const;
    
    protected:
        camera_calib left_, right_;
        cv::Matx44d Q_;
        void updateQ();
};

inline const camera_calib StereoModel::left(){ return left_;}
inline const camera_calib StereoModel::right(){ return right_;}
inline const cv::Matx44d& StereoModel::reprojectionMatrix() { return Q_;}
inline double StereoModel::baseline() const
{
    return -right_.Tx / right_.fx;
}

#endif