#include "image_process/Camera_Model.h"

StereoModel::StereoModel():Q_(4,4,0.0)
{
    Q_(0,0) = Q_(1,1) = 1.0;
}

StereoModel::~StereoModel(){}

bool StereoModel::fromCameraCalib(const camera_calib &left, const camera_calib &right)
{
    left_ = left;
    right_ = right;

    assert(left_.fx == right_.fx);
    assert(left_.fy == right_.fy);
    assert(left_.cy == right_.cy);
    
    //cx may differ for verged cameras

    updateQ();

    return true;
}

void StereoModel::projectDisparityTo3d(const cv::Point2d& left_uv_rect, float disparity, cv::Point3d& xyz) const
{
    double u = left_uv_rect.x, v = left_uv_rect.y;
    cv::Point3d XYZ(u + Q_(0,3), v + Q_(1,3), Q_(2,3));
    double W = Q_(3,2)*disparity + Q_(3,3);
    xyz = XYZ * (1.0/W);
}

void StereoModel::updateQ()
{
    double Tx = baseline();

    Q_(3,2) = 1.0 / Tx;
    Q_(0,3) = -right_.cx;
    Q_(1,3) = -right_.cy;
    Q_(2,3) = right_.fx;
}