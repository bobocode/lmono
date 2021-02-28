#ifndef _MONO_RESIDUAL_INFO_H_
#define _MONO_RESIDUAL_INFO_H_

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <sophus/so3.h>
#include <sophus/se3.h>
//#include <cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>

#include "PoseLocalParameterization.h"

class MonoProjectionFactor: public ceres::SizedCostFunction<2,7,7,7,1>
{
    public:
        ~MonoProjectionFactor(){};
        MonoProjectionFactor(const Eigen::Vector2d &pt_i, const Eigen::Vector2d &pt_j, const Eigen::Vector2d &velocity_i,
                     const Eigen::Vector2d &velocity_j, const double td_i, const double td_j);
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