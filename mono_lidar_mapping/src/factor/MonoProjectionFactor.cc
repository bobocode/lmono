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
#include "factor/MonoProjectionFactor.h"

Eigen::Matrix2d MonoProjectionFactor::sqrt_info;

MonoProjectionFactor::MonoProjectionFactor(const Eigen::Vector2d &pt_i, const Eigen::Vector2d &pt_j, const Eigen::Vector2d &velocity_i,
                     const Eigen::Vector2d &velocity_j, const double td_i, const double td_j): 
                    _td_i(td_i), _td_j(td_j)
{
    _pt_i.x() = pt_i.x();
    _pt_i.y() = pt_i.y();
    _pt_i.z() = 1.0;

    _velocity_i.x() = velocity_i.x();
    _velocity_i.y() = velocity_i.y();
    _velocity_i.z() = 0.0;

    _pt_j.x() = pt_j.x();
    _pt_j.y() = pt_j.y();
    _pt_j.z() = 1.0;

    _velocity_j.x() = velocity_j.x();
    _velocity_j.y() = velocity_j.y();
    _velocity_j.z() = 0.0;


    Eigen::Vector3d b1, b2;
    Eigen::Vector3d a = _pt_j.normalized();
    Eigen::Vector3d tmp(0, 0, 1);
    if(a == tmp)
        tmp << 1, 0, 0;
    b1 = (tmp - a * (a.transpose() * tmp)).normalized();
    b2 = a.cross(b1);
    tangent_base.block<1, 3>(0, 0) = b1.transpose();
    tangent_base.block<1, 3>(1, 0) = b2.transpose();

};

bool MonoProjectionFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Map<Eigen::Matrix<double, 2,1>> residual(residuals); 

    Eigen::Quaterniond Qx(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
    Eigen::Vector3d tx(parameters[0][0], parameters[0][1], parameters[0][2]);

    Eigen::Vector3d ti(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Vector3d tj(parameters[2][0], parameters[2][1], parameters[2][2]);

    Eigen::Quaterniond Qi(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);
    Eigen::Quaterniond Qj(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

    double depth_inv = parameters[3][0];

    double depth = 1.0 / depth_inv;

    Eigen::Matrix3d Ri = Qi.normalized().toRotationMatrix();
    Eigen::Matrix3d Rj = Qj.normalized().toRotationMatrix();
    Eigen::Matrix3d Rlc = Qx.normalized().toRotationMatrix();

    Eigen::Vector3d pt_td_i = _pt_i; //- _td_i * _velocity_i;
    Eigen::Vector3d pt_td_j = _pt_j;// - _td_j * _velocity_j;

    Eigen::Vector3d pts_ci = depth * pt_td_i;
    Eigen::Vector3d pts_laser_i = Qx * pts_ci + tx;
    Eigen::Vector3d pts_w = Qi * pts_laser_i + ti;
    Eigen::Vector3d pt_l_j = Qj.inverse() * (pts_w - tj);
    Eigen::Vector3d pts_cj = Qx.inverse() * (pt_l_j - tx);
    double dep_cj = pts_cj.z();

// #ifdef UNIT_SPHERE_ERROR 
//     residual = tangent_base * (pts_cj.normalized() - pt_td_j.normalized());
// #else
//     residual = (pts_cj / dep_cj).head<2>() - pt_td_j.head<2>();
// #endif
    residual = (pts_cj / dep_cj).head<2>() - pt_td_j.head<2>();
    residual = sqrt_info *residual;

    // std::cout << "depth inv: " << depth_inv << std::endl;

    //std::cout << "depth: " << depth << std::endl;

    // std::cout << "tx: " << tx.transpose() << std::endl;

    // std::cout << "Qx: " << Qx.toRotationMatrix() << std::endl;

    // std::cout << "pts_w: " << pts_w << std::endl;

    //std::cout << "pts_ci: " << pts_ci << std::endl;

    //std::cout << "pts_cj: " << pts_ci << std::endl;

    //std::cout << "mono two frame residual: " << residual.transpose() << std::endl;

    Eigen::Matrix<double ,2,3> reduce(2,3);

// #ifdef UNIT_SPHERE_ERROR
//     double norm = pts_cj.norm();
//     Eigen::Matrix3d norm_jaco;
//     double x1, x2, x3;
//     x1 = pts_cj(0);
//     x2 = pts_cj(1);
//     x3 = pts_cj(2);
//     norm_jaco << 1.0 / norm - x1 * x1 / pow(norm, 3), - x1 * x2 / pow(norm, 3),            - x1 * x3 / pow(norm, 3),
//                     - x1 * x2 / pow(norm, 3),            1.0 / norm - x2 * x2 / pow(norm, 3), - x2 * x3 / pow(norm, 3),
//                     - x1 * x3 / pow(norm, 3),            - x2 * x3 / pow(norm, 3),            1.0 / norm - x3 * x3 / pow(norm, 3);
//     reduce = tangent_base * norm_jaco;
// else
//     reduce << 1. /dep_cj, 0, -pts_cj(0) / (dep_cj * dep_cj),
//             0, 1. /dep_cj, -pts_cj(1) /(dep_cj * dep_cj);
// #endif
    double norm = pts_cj.norm();
    Eigen::Matrix3d norm_jaco;
    double x1, x2, x3;
    x1 = pts_cj(0);
    x2 = pts_cj(1);
    x3 = pts_cj(2);
    norm_jaco << 1.0 / norm - x1 * x1 / pow(norm, 3), - x1 * x2 / pow(norm, 3),            - x1 * x3 / pow(norm, 3),
                    - x1 * x2 / pow(norm, 3),            1.0 / norm - x2 * x2 / pow(norm, 3), - x2 * x3 / pow(norm, 3),
                    - x1 * x3 / pow(norm, 3),            - x2 * x3 / pow(norm, 3),            1.0 / norm - x3 * x3 / pow(norm, 3);
    reduce = tangent_base * norm_jaco;
    

    reduce = sqrt_info * reduce;

    if(jacobians)
    {
        
        if(jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double,2,7, Eigen::RowMajor>> j_ex(jacobians[0]);
            Eigen::Matrix<double,3,6> jaco;

            jaco.leftCols<3>() =  Rlc.transpose() * ((Rj.transpose()* Ri).normalized() - Eigen::Matrix3d::Identity());

            Eigen::Matrix3d temp_r = Rlc.transpose() * Rj.transpose() * Ri * Rlc;

            jaco.rightCols<3>() = - temp_r * mathutils::SkewSymmetric(pts_ci) + mathutils::SkewSymmetric(temp_r * pts_ci) + 
                            mathutils::SkewSymmetric(Rlc.transpose() * (Rj.transpose() * (Ri * tx + ti - tj)-tx));
            //jaco.rightCols<3>().setZero();
            j_ex.leftCols<6>() = reduce * jaco;
            j_ex.rightCols<1>().setZero();
           
        }

        if(jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double,2,7,Eigen::RowMajor>> j_i(jacobians[1]);
            Eigen::Matrix<double,3,6> jaco_i;

            jaco_i.leftCols<3>() = Rlc.transpose() * Rj.transpose();
            jaco_i.rightCols<3>() = Rlc.transpose() * Rj.transpose() * Ri * -mathutils::SkewSymmetric(pts_laser_i);
            
            j_i.leftCols<6>()= reduce * jaco_i;
            j_i.rightCols<1>().setZero();

        }

        if(jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double,2,7,Eigen::RowMajor>> j_j(jacobians[2]);
            Eigen::Matrix<double,3,6> jaco_j;

            jaco_j.leftCols<3>() = Rlc.transpose() * (-1) * Rj.transpose();
            jaco_j.rightCols<3>() = Rlc.transpose() * mathutils::SkewSymmetric(pt_l_j);
            
            j_j.leftCols<6>()= reduce * jaco_j;
            j_j.rightCols<1>().setZero();
        }

        if(jacobians[3])
        {

            Eigen::Map<Eigen::Vector2d> j_depth(jacobians[3]);
            j_depth = (-1) * reduce * Rlc.transpose() * Rj.transpose() * Ri * Rlc * pt_td_i * depth * depth;
            //std::cout << "depth inv: " << depth_inv << std::endl;
            //std::cout << "jaco depth:\n" << j_depth << std::endl;
        }

    }

    return true;
}
