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

#ifndef _PRIORFACTOR_H_
#define _PRIORFACTOR_H_

#include <ceres/ceres.h>
#include "utils/math_utils.h"
#include "parameter.h"


using namespace mathutils;

class PriorFactor : public ceres::SizedCostFunction<6, 7> {

 public:
    PriorFactor() = delete;
    PriorFactor(const Eigen::Matrix4d &transform)
    {
        pos_ = transform.block<3,1>(0,3);
        rot_ = transform.block<3,3>(0,0);

    }

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        Eigen::Vector3d P{parameters[0][0], parameters[0][1], parameters[0][2]};
        Eigen::Quaterniond Q(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

        Eigen::Matrix<double, 6, 6> sqrt_info;
        sqrt_info.setZero();
        sqrt_info.topLeftCorner<3, 3>() = Matrix3d::Identity() * PRIOR_T;
        sqrt_info.bottomRightCorner<3, 3>() = Matrix3d::Identity() * PRIOR_R;

        Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
        residual.topRows<3>() = P - pos_;
        residual.bottomRows<3>() = 2 * (rot_.inverse() * Q).coeffs().head<3>();

        residual = sqrt_info * residual;

        if (jacobians) {
            if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor> > jacobian_prior(jacobians[0]);
            Eigen::Matrix<double, 6, 6> jaco_prior;
            jaco_prior.setIdentity();

            jaco_prior.bottomRightCorner<3, 3>() = LeftQuatMatrix(Q.inverse() * rot_).topLeftCorner<3, 3>();

            // FIXME: info
            jacobian_prior.setZero();
            jacobian_prior.leftCols<6>() = sqrt_info * jaco_prior;
            jacobian_prior.rightCols<1>().setZero();
            }
        }

        return true;

    }

    Eigen::Vector3d pos_;
    Eigen::Quaterniond rot_;

};


#endif //_PRIORFACTOR_H_
