#ifndef _AXXB_SOLVER_H_
#define _AXXB_SOLVER_H_

#include <vector>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "../utils/math_utils.h"

using namespace Eigen;
using namespace std;

class AXXBSolver
{
    public:
      void InitialAXXBSolver();
      bool CalibrationExRotation(Quaterniond delta_q_camera, Quaterniond delta_q_lidar, Matrix3d &calib_rlc_result,int count);
      Eigen::Matrix3d solveRelativeR(const vector<pair<Vector3d, Vector3d>> &corres);
      void decomposeE(cv::Mat E,
                    cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                    cv::Mat_<double> &t1, cv::Mat_<double> &t2);
      double testTriangulation(const vector<cv::Point2f> &l,
                             const vector<cv::Point2f> &r,
                             cv::Mat_<double> R, cv::Mat_<double> t);
                             
    private:
        int frame_count;

        vector<Matrix3d> Rc;
        vector<Matrix3d> Rlidar;
        vector<Matrix3d> Rc_g;
        Matrix3d rlc;

};

#endif