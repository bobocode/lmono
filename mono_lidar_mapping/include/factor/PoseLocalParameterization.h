#ifndef _POSELOCALPARAMETERIZATION_H_
#define _POSELOCALPARAMETERIZATION_H_

#include <ceres/ceres.h>
#include <Eigen/Eigen>

#include "utils/math_utils.h"


using namespace mathutils;

class PoseLocalParameterization : public ceres::LocalParameterization {

  virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
  virtual bool ComputeJacobian(const double *x, double *jacobian) const;
  virtual int GlobalSize() const { return 7; };
  virtual int LocalSize() const { return 6; };

};

#endif //
