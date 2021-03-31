#ifndef __REPROJECTION_FACTOR_H_
#define __REPROJECTION_FACTOR_H_

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
//#include <cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include "parameter.h"

class ReprojectionFactor: public ceres::SizedCostFunction<2,1>
{
    public:
        ~ReprojectionFactor(){};
        ReprojectionFactor(const Eigen::Vector2d &pt_i, const Eigen::Vector2d &pt_j, const Eigen::Matrix3d &Ri, 
                        const Eigen::Vector3d &Pi, const Eigen::Matrix3d &Rj, const Eigen::Vector3d &Pj, Eigen::Matrix4d &EX)
        {
            _pt_i.x() = pt_i.x();
            _pt_i.y() = pt_i.y();
            _pt_i.z() = 1.0;

            _pt_j.x() = pt_j.x();
            _pt_j.y() = pt_j.y();
            _pt_j.z() = 1.0;

            _Ri = Ri;
            _Rj = Rj;
            _Pi = Pi;
            _Pj = Pj;

            Rlc = EX.block<3,3>(0,0);
            Tlc = EX.block<3,1>(0,3);

            sqrt_info = FACTOR_WEIGHT * Eigen::Matrix2d::Identity();
        }

        virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
        {
            double inv_dep = parameters[0][0];

            Eigen::Map<Eigen::Matrix<double, 2,1>> residual(residuals);

            double dep = 1.0 / inv_dep;

            Eigen::Vector3d pts_ci = dep * _pt_i;
            Eigen::Vector3d pts_li = Rlc * pts_ci + Tlc;

            Eigen::Vector3d pts_w = _Ri * pts_li + _Pi;
            Eigen::Vector3d pts_lj = _Rj.transpose() * (pts_w - _Pj);

            Eigen::Vector3d pts_cj = Rlc.transpose() * (pts_lj - Tlc);

            double dep_cj = pts_cj.z();

            residual = (pts_cj / dep_cj).head<2>() - _pt_j.head<2>();
            residual = sqrt_info *residual;

            Eigen::Matrix<double ,2,3> reduce(2,3);

            reduce << 1. /dep_cj, 0, -pts_cj(0) / (dep_cj * dep_cj),
            0, 1. /dep_cj, -pts_cj(1) /(dep_cj * dep_cj);
    
            reduce = sqrt_info * reduce;

            if(jacobians)
            {
                if(jacobians[0])
                {
                    Eigen::Map<Eigen::Vector2d> j_depth(jacobians[0]);
                    j_depth = (-1) * reduce * Rlc.transpose() * _Rj.transpose() * _Ri * Rlc * _pt_i * dep * dep;
                }
            }

            return true;

        }

        void check(double **parameters);

        //Eigen::Matrix4d Pi;
        //Eigen::Matrix4d Pj;
        Eigen::Vector3d _pt_i;
        Eigen::Vector3d _pt_j;
        Eigen::Matrix3d _Ri;
        Eigen::Matrix3d _Rj;
        Eigen::Vector3d _Pi;
        Eigen::Vector3d _Pj;
        Eigen::Matrix3d Rlc;
        Eigen::Vector3d Tlc;

        Eigen::Matrix2d sqrt_info;
        Eigen::Matrix<double, 2, 3> tangent_base;
};

#endif