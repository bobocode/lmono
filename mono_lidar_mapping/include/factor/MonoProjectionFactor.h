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

//Adapted from VINS-mono
*******************************************************/

#ifndef _MONO_RESIDUAL_INFO_H_
#define _MONO_RESIDUAL_INFO_H_

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
//#include <cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>

#include "PoseLocalParameterization.h"

class MonoProjectionFactor: public ceres::SizedCostFunction<2,7,7,7,1>
{
    public:
        ~MonoProjectionFactor(){};
        MonoProjectionFactor(const Eigen::Vector2d &pt_i, const Eigen::Vector2d &pt_j);
        virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
        void check(double **parameters);

        //Eigen::Matrix4d Pi;
        //Eigen::Matrix4d Pj;
        Eigen::Vector3d _pt_i;
        Eigen::Vector3d _pt_j;
        Eigen::Vector3d _velocity_i;
        Eigen::Vector3d _velocity_j;

        double _td_i;
        double _td_j;
        static Eigen::Matrix2d sqrt_info;
        Eigen::Matrix<double, 2, 3> tangent_base;
};

#endif