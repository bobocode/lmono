/*******************************************************
* Copyright (C) 2020, Intelligent Positioning and Navigation Lab, Hong Kong Polytechnic University
*
* This file is part of lclio.
* Licensed under the GNU General Public License v3.0;
* you may not use this file except in compliance with the License.
*
* Author: Bo Zhang (bo1zhang@polyu.edu.hk)
* Main fucntions: Implementation of pose state used in factor graph optimization. The file is modified from VINS-mono.
* Date: 2020/11/12
*******************************************************/

#include "factor/PoseLocalParameterization.h"

bool PoseLocalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
  Eigen::Map<const Eigen::Vector3d> p(x);
  Eigen::Map<const Eigen::Quaterniond> q(x + 3);

  Eigen::Map<const Eigen::Vector3d> dp(delta);

  Eigen::Quaterniond dq = DeltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

  Eigen::Map<Eigen::Vector3d> p_plus(x_plus_delta);
  Eigen::Map<Eigen::Quaterniond> q_plus(x_plus_delta + 3);

  p_plus = p + dp;
  q_plus = (q * dq).normalized();

  return true;
}

bool PoseLocalParameterization::ComputeJacobian(const double *x, double *jacobian) const
{
  Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
  j.topRows<6>().setIdentity();
  j.bottomRows<1>().setZero();

  return true;
}
