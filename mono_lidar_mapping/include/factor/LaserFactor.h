#ifndef _LASER_FACTOR_H_
#define _LASER_FACTOR_H_

#include <ros/assert.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "utils/math_utils.h"
#include "parameter.h"

class LASERFactor : public ceres::SizedCostFunction<6,7,7>
{
  public:
    LASERFactor() = delete;
    LASERFactor(Eigen::Matrix3d &_L0_Ri, Eigen::Matrix3d &_L0_Rj, Eigen::Vector3d &_L0_Pi, Eigen::Vector3d &_L0_Pj)
    {
        L0_Ri = _L0_Ri;
        L0_Pi = _L0_Pi;

        L0_Rj = _L0_Rj;
        L0_Pj = _L0_Pj;

        delta_ij = Eigen::Quaterniond((L0_Ri.transpose() * L0_Rj));
        delta_pij =  L0_Ri.transpose() * (L0_Pj - L0_Pi);

        noise.setZero();
        noise.block<3,3>(0,0) = (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
        noise.block<3,3>(3,3) = (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();

    }
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
        Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);

        Eigen::Matrix3d Ri = Qi.toRotationMatrix();
        Eigen::Matrix3d Rj = Qj.toRotationMatrix();

        Eigen::Map<Eigen::Matrix<double, 6,1>> residual(residuals);

        residual.block<3,1>(0,0) = Qi.inverse() *(Pj - Pi) - delta_pij;
        residual.block<3,1>(3,0) = 2 * (delta_ij.inverse() * (Qi.inverse() * Qj)).vec();

        //std::cout << "delta_ij: " << delta_ij.vec().transpose() << std::endl;
        //std::cout << (Qi.inverse().normalized() * Qj.normalized()).vec() << std::endl;

        //std::cout << "dp: " << delta_pij.transpose() << std::endl;
        //std::cout << Qi.inverse() * (Pj - Pi) << std::endl;
        sqrt_info = sqrt_info;// + noise;

        residual = sqrt_info * residual;

        if(jacobians)
        {
            if(jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
                jacobian_pose_i.setZero();

                jacobian_pose_i.block<3,3>(0,0) = -Qi.inverse().toRotationMatrix();
                jacobian_pose_i.block<3,3>(0,3) = mathutils::SkewSymmetric(Qi.inverse() * (Pj - Pi));

                jacobian_pose_i.block<3,3>(3,3) = -(mathutils::LeftQuatMatrix(Qj.inverse() * Qi) * mathutils::RightQuatMatrix(delta_ij)).bottomRightCorner<3, 3>();

                jacobian_pose_i = sqrt_info * jacobian_pose_i;
            }

            if(jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);
                jacobian_pose_j.setZero();

                jacobian_pose_j.block<3,3>(0,0) = Qi.inverse().toRotationMatrix();
                jacobian_pose_j.block<3,3>(3,3) = mathutils::LeftQuatMatrix(delta_ij.inverse() * Qi.inverse()* Qj).bottomRightCorner<3, 3>();

                jacobian_pose_j = sqrt_info * jacobian_pose_j;
            }

        }

        return true;

    }

    Eigen::Matrix3d L0_Ri;
    Eigen::Vector3d L0_Pi;

    Eigen::Matrix3d L0_Rj;
    Eigen::Vector3d L0_Pj;

    Eigen::Quaterniond delta_ij;
    Eigen::Vector3d delta_pij;

    static Eigen::Matrix<double, 6,6> sqrt_info;
    Eigen::Matrix<double,6,6> noise;
};

Eigen::Matrix<double,6,6> LASERFactor::sqrt_info;


#endif