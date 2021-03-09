/**
* This file is part of LIO-mapping.
* 
* Copyright (C) 2019 Haoyang Ye <hy.ye at connect dot ust dot hk>,
* Robotics and Multiperception Lab (RAM-LAB <https://ram-lab.com>),
* The Hong Kong University of Science and Technology
* 
* For more information please see <https://ram-lab.com/file/hyye/lclio-mapping>
* or <https://sites.google.com/view/lclio-mapping>.
* If you use this code, please cite the respective publications as
* listed on the above websites.
* 
* LIO-mapping is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* LIO-mapping is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with LIO-mapping.  If not, see <http://www.gnu.org/licenses/>.
*/

//
// Created by hyye on 3/14/18.
//

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

#include "laser_odometry/PointProcessor.h"

//#define DEBUG
//#define DEBUG_ORIGIN

namespace lclio
{

  double AbsRadDistance(double a, double b)
  {
    return fabs(NormalizeRad(a - b));
  }

  PointProcessor::PointProcessor() : PointProcessor(-15.0, 15.0, 16) {}

  PointProcessor::PointProcessor(float lower_bound, float upper_bound, int num_rings, bool uneven)
      : lower_bound_(lower_bound),
        upper_bound_(upper_bound),
        num_rings_(num_rings),
        factor_((num_rings - 1) / (upper_bound - lower_bound)),
        uneven_(uneven)
  {
    PointCloud scan;
    laser_scans.clear();
    for (int i = 0; i < num_rings_; ++i)
    {
      PointCloudPtr scan(new PointCloud());
      laser_scans.push_back(scan);
    }

    intensity_scans.clear();
    for (int i = 0; i < num_rings_; ++i)
    {
      PointCloudPtr scan(new PointCloud());
      intensity_scans.push_back(scan);
    }
  }

  void PointProcessor::Process()
  {
    PointToRing();
    ExtractFeaturePoints();
    PublishResults();
  }

  // void PointProcessor::PointCloudHandler(const sensor_msgs::PointCloud2ConstPtr &raw_points_msg) {

  //   // fetch new input cloud
  //   if (uneven_ == false) {
  //     PointCloud laser_cloud_in;
  //     pcl::fromROSMsg(*raw_points_msg, laser_cloud_in);
  //     PointCloudConstPtr laser_cloud_in_ptr(new PointCloud(laser_cloud_in));

  //     SetInputCloud(laser_cloud_in_ptr, raw_points_msg->header.stamp);
  //   } else {
  //     pcl::PointCloud<PointIR> laser_cloud_in;
  //     pcl::fromROSMsg(*raw_points_msg, laser_cloud_in);
  //     pcl::PointCloud<PointIR>::Ptr laser_cloud_in_ptr(new pcl::PointCloud<PointIR>(laser_cloud_in));

  //     SetInputCloud(laser_cloud_in_ptr, raw_points_msg->header.stamp);
  //   }
  //   Process();
  // }

  // void PointProcessor::LCOdometryHandler(const nav_msgs::Odometry::ConstPtr &laser_odom_msg){
  //   LOG(INFO)<< std::setprecision(18)<<"PointProcessor::LCOdometryHandler"<<laser_odom_msg->header.stamp.toSec()<<std::endl;
  // }


  void PointProcessor::ImuHandler(const sensor_msgs::Imu::ConstPtr &imuMsg)
  {

    //todo convert imu to lidar frame
    // sensor_msgs::Imu thisImu = imuConverter(*imuMsg);
    sensor_msgs::Imu thisImu = imuConverter(*imuMsg);

    //   std::lock_guard<std::mutex> lock1(imuLock);
    imu_queue_.push_back(*imuMsg);

    // // debug IMU data
    // cout << std::setprecision(6);
    // if(imu_queue_.size() > 2000){
    //     cout << "too much IMU, size: " << imu_queue_.size() << endl;
    // }
  }


  void PointProcessor::ImuOdomHandler(const nav_msgs::Odometry::ConstPtr &odomMsg)
  {
    odom_queue_.push_back(*odomMsg);
    cout << std::setprecision(6);
    if(odom_queue_.size() > 2000){
       // cout << "too much IMU odom, size: " << imu_queue_.size() << endl;
        odom_queue_.pop_front();
    }
  }


  void PointProcessor::PointCloudHandler(const sensor_msgs::PointCloud2ConstPtr &raw_points_msg)
  {

    cloud_queue_.push_back(*raw_points_msg);
    //cache queue to make sure the imu data is receive fully
    if (cloud_queue_.size() <= 2)
      return;
    auto current_cloud = cloud_queue_.front();

    cloud_queue_.pop_front();
    // fetch new input cloud

    if (uneven_ == false)
    {
      PointCloud laser_cloud_in;
      pcl::fromROSMsg(current_cloud, laser_cloud_in);
      PointCloudConstPtr laser_cloud_in_ptr(new PointCloud(laser_cloud_in));

      SetInputCloud(laser_cloud_in_ptr, current_cloud.header.stamp);
    }
    else
    {
      // pcl::PointCloud<PointIR> laser_cloud_in;
      // pcl::fromROSMsg(current_cloud, laser_cloud_in);
      // pcl::PointCloud<PointIR>::Ptr laser_cloud_in_ptr(new pcl::PointCloud<PointIR>(laser_cloud_in));

      // SetInputCloud(laser_cloud_in_ptr, current_cloud.header.stamp);

      pcl::PointCloud<PointIRT> laser_cloud_in;
      pcl::fromROSMsg(current_cloud, laser_cloud_in);
      pcl::PointCloud<PointIRT>::Ptr laser_cloud_in_ptr(new pcl::PointCloud<PointIRT>(laser_cloud_in));
      // check point time
      deskew_flag = false;
      for (int i = 0; i < (int)current_cloud.fields.size(); ++i)
      {
        if (current_cloud.fields[i].name == "time" && frame_count > 500)  //more than 50s
        {
          deskew_flag = true;
          time_scan_start_ = current_cloud.header.stamp.toSec();
          time_scan_end_ = time_scan_start_ + laser_cloud_in_ptr->points.back().time;
          // std::cout<<std::setprecision(18)<<" time_scan_start_"<<time_scan_start_<<" "<<time_scan_end_<<std::endl;
          ImuDeskewInfo();
          OdomDeskewInfo();
          break;
        }
      }
      if (deskew_flag == false)
        ROS_WARN("Point cloud timestamp not available, deskew function disabled!");

      SetInputCloud(laser_cloud_in_ptr, current_cloud.header.stamp);
      
    }
    Process();

    //reset deskew 
    imu_pointer_cur = 0;
    firstPointFlag = true;
    odomDeskewFlag = false;

    for (int i = 0; i < queue_length; ++i)
    {
        imu_time[i] = 0;
        imu_rotX[i] = 0;
        imu_rotY[i] = 0;
        imu_rotZ[i] = 0;
    }
    frame_count ++;
  } // namespace lclio

  void PointProcessor::SetupRos(ros::NodeHandle &nh)
  {

    is_ros_setup_ = true;

    // subscribe to raw cloud topic
    sub_raw_points_ = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 2, &PointProcessor::PointCloudHandler, this);
    sub_raw_imu_ = nh.subscribe<sensor_msgs::Imu>("/imu/data", 2000, &PointProcessor::ImuHandler, this, ros::TransportHints().tcpNoDelay());
    sub_hf_odom_ = nh.subscribe<nav_msgs::Odometry>("/imu/data/incremental", 2000, &PointProcessor::ImuOdomHandler, this,ros::TransportHints().tcpNoDelay());

   // sub_lc_odom_ = nh.subscribe<nav_msgs::Odometry>("/high_freq_predict_odom_feedback", 500, &PointProcessor::LCOdometryHandler, this);
    // advertise scan registration topics
    pub_full_cloud_ = nh.advertise<sensor_msgs::PointCloud2>("/full_cloud", 2);
    pub_deskew_cloud_ = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_deskew", 2);

    pub_corner_points_sharp_ = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 2);
    pub_corner_points_less_sharp_ = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 2);
    pub_surf_points_flat_ = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 2);
    pub_surf_points_less_flat_ = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 2);

    pub_start_ori_ = nh.advertise<std_msgs::Float32>("/debug/start_ori", 10);
    pub_start_ori_inferred_ = nh.advertise<std_msgs::Float32>("/debug/start_ori_inferred", 10);
    frame_count = 0;

  }

  void PointProcessor::Reset(const ros::Time &scan_time, const bool &is_new_sweep)
  {
    scan_time_ = scan_time;

    // clear internal cloud buffers at the beginning of a sweep
    if (is_new_sweep)
    {
      sweep_start_ = scan_time_;

      // clear cloud buffers
      cloud_in_rings_.clear();
      corner_points_sharp_.clear();
      corner_points_less_sharp_.clear();
      surface_points_flat_.clear();
      surface_points_less_flat_.clear();

      // clear scan indices vector
      scan_ranges.clear();

      for (int i = 0; i < laser_scans.size(); ++i)
      {
        laser_scans[i]->clear();
      }

      for (int i = 0; i < intensity_scans.size(); ++i)
      {
        intensity_scans[i]->clear();
      }

      //    laser_scans.clear();
      //
      //    for (int i = 0; i < num_rings_; ++i) {
      //      PointCloudPtr scan(new PointCloud());
      //      laser_scans.push_back(scan);
      //    }
    }
  }

  void PointProcessor::SetInputCloud(const PointCloudConstPtr &cloud_in, ros::Time time_in)
  {
    Reset(time_in);
    cloud_ptr_ = cloud_in;
  }

  void PointProcessor::SetInputCloud(const pcl::PointCloud<PointIR>::Ptr &cloud_in, ros::Time time_in)
  {
    Reset(time_in);
    cloud_ir_ptr_ = cloud_in;
  }

  void PointProcessor::SetInputCloud(const pcl::PointCloud<PointIRT>::Ptr &cloud_in, ros::Time time_in)
  {
    Reset(time_in);
    cloud_irt_ptr_ = cloud_in;
  }

  void PointProcessor::PointToRing()
  {
    if (uneven_ == false)
    {
      PointToRing(cloud_ptr_, laser_scans, intensity_scans);
    }
    else
    {
      PointToRing(cloud_irt_ptr_, laser_scans, intensity_scans);
    }

    // construct sorted full resolution cloud
    size_t cloud_size = 0;
    for (int i = 0; i < num_rings_; i++)
    {
      cloud_in_rings_ += (*intensity_scans[i]);

      IndexRange range(cloud_size, 0);
      cloud_size += (*laser_scans[i]).size();
      range.second = (cloud_size > 0 ? cloud_size - 1 : 0);
      scan_ranges.push_back(range);
    }

    ROS_DEBUG_STREAM("point size: " << cloud_in_rings_.size());
  }

  void PointProcessor::ImuDeskewInfo()
  {
    imu_available = false;

    while (!imu_queue_.empty())
    {
      //LOG(ERROR) << "imu_queue_.size()" << imu_queue_.size() << std::endl;
      if (imu_queue_.front().header.stamp.toSec() < time_scan_start_ - 0.01)
        imu_queue_.pop_front();
      else
        break;
    }

    if (imu_queue_.empty())
      return;

    imu_pointer_cur = 0;

    for (int i = 0; i < (int)imu_queue_.size(); ++i)
    {
      sensor_msgs::Imu thisImuMsg = imu_queue_[i];
      double currentImuTime = thisImuMsg.header.stamp.toSec();

      // get roll, pitch, and yaw estimation for this scan
      // if (currentImuTime <= time_scan_start_)
      //   imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);

      if (currentImuTime > time_scan_end_ + 0.01)
        break;

      if (imu_pointer_cur == 0)
      {
        imu_rotX[0] = 0;
        imu_rotY[0] = 0;
        imu_rotZ[0] = 0;
        imu_time[0] = currentImuTime;
        ++imu_pointer_cur;
        continue;
      }

      // get angular velocity
      double angular_x, angular_y, angular_z;
      imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);
      angular_z = -angular_z;
      // integrate rotation
      double timeDiff = currentImuTime - imu_time[imu_pointer_cur - 1];
      imu_rotX[imu_pointer_cur] = imu_rotX[imu_pointer_cur - 1] + angular_x * timeDiff;
      imu_rotY[imu_pointer_cur] = imu_rotY[imu_pointer_cur - 1] + angular_y * timeDiff;
      imu_rotZ[imu_pointer_cur] = imu_rotZ[imu_pointer_cur - 1] + angular_z * timeDiff;
      imu_time[imu_pointer_cur] = currentImuTime;
     // std::cout<< " imu_rotZ[imu_pointer_cur] "<<  imu_rotZ[imu_pointer_cur] <<std::endl;
      //std::cout<< std::setprecision(18)<<" imu_time[imu_pointer_cur] "<<  imu_time[imu_pointer_cur] <<std::endl;

      ++imu_pointer_cur;
     

    }

    --imu_pointer_cur;

    if (imu_pointer_cur <= 0)
      return;
    //std::cout<< "ImuDeskewInfo imu_pointer_cur:"<< imu_pointer_cur<<std::endl;
    imu_available = true;
  }


  void PointProcessor::OdomDeskewInfo() {
    odom_available = false;

    while (!odom_queue_.empty())
    {
        if (odom_queue_.front().header.stamp.toSec() < time_scan_start_ - 0.01)
            odom_queue_.pop_front();
        else
            break;
    }

    if (odom_queue_.empty())
        return;

    if (odom_queue_.front().header.stamp.toSec() > time_scan_start_)
        return;

    // get start odometry at the beinning of the scan
    nav_msgs::Odometry startOdomMsg;

    for (int i = 0; i < (int)odom_queue_.size(); ++i)
    {
        startOdomMsg = odom_queue_[i];

        if (startOdomMsg.header.stamp.toSec()  < time_scan_start_)
            continue;
        else
            break;
    }

    tf::Quaternion orientation;
    tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

    double roll, pitch, yaw;
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    if (odom_queue_.back().header.stamp.toSec() < time_scan_end_)
        return;

    nav_msgs::Odometry endOdomMsg;

    for (int i = 0; i < (int)odom_queue_.size(); ++i)
    {
        endOdomMsg = odom_queue_[i];

        if (endOdomMsg.header.stamp.toSec() < time_scan_end_)
            continue;
        else
            break;
    }

    if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
        return;

    Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

    tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

    Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

   // float rollIncre, pitchIncre, yawIncre;
    pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

    odom_available= true;
    LOG(INFO)<< "odom_available" << odom_available <<" "<<odomIncreX<<" "<<odomIncreY<<" "<<odomIncreZ<<std::endl;
    LOG(INFO)<< "odom_available" << odom_available <<" "<<rollIncre<<" "<<pitchIncre<<" "<<yawIncre<<std::endl;

  }

  void PointProcessor::FindRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
  {
    *rotXCur = 0;
    *rotYCur = 0;
    *rotZCur = 0;

  //   float ratio = relTime / (time_scan_end_ - time_scan_start_);
  //  //  std::cout<<std::setprecision(18)<<"rotation ratio"<<pointTime<<" time_scan_end_"<<time_scan_end_<<" time_scan_start_"<<time_scan_start_<<std::endl;

  //   *rotXCur = ratio * rollIncre;
  //   *rotYCur = ratio * pitchIncre;
  //   *rotZCur = ratio * yawIncre;

    int imuPointerFront = 0;
    while (imuPointerFront < imu_pointer_cur)
    {
      if (pointTime < imu_time[imuPointerFront]){
          break;

      }

      ++imuPointerFront;
    }
     // if empty IMU still no data
    if (pointTime > imu_time[imuPointerFront] || imuPointerFront == 0)
    {
      *rotXCur = imu_rotX[imuPointerFront];
      *rotYCur = imu_rotY[imuPointerFront];
      *rotZCur = imu_rotZ[imuPointerFront];
    }
    else
    {
      int imuPointerBack = imuPointerFront - 1;
      double ratioFront = (pointTime - imu_time[imuPointerBack]) / (imu_time[imuPointerFront] - imu_time[imuPointerBack]);
      double ratioBack = (imu_time[imuPointerFront] - pointTime) / (imu_time[imuPointerFront] - imu_time[imuPointerBack]);
      *rotXCur = imu_rotX[imuPointerFront] * ratioFront + imu_rotX[imuPointerBack] * ratioBack;
      *rotYCur = imu_rotY[imuPointerFront] * ratioFront + imu_rotY[imuPointerBack] * ratioBack;
      *rotZCur = imu_rotZ[imuPointerFront] * ratioFront + imu_rotZ[imuPointerBack] * ratioBack;
     // std::cout << " rotXCur:" << *rotXCur << " rotYCur:" << *rotYCur << std::endl;
    }
  }

  void PointProcessor::FindPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
  {
      *posXCur = 0; *posYCur = 0; *posZCur = 0;

      // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.

      if (odom_available == false)
          return;

      float ratio = relTime / (time_scan_end_ - time_scan_start_);

      *posXCur = ratio * odomIncreX;
      *posYCur = ratio * odomIncreY;
      *posZCur = ratio * odomIncreZ;
  }

  void PointProcessor::DeSkew(PointT &point, double rel_time)
  {
    if (deskew_flag == false || !imu_available)
      return;
    return;
    double point_time = time_scan_start_ + rel_time;

    float rotXCur = 0, rotYCur =0, rotZCur=0;
    FindRotation(point_time, &rotXCur, &rotYCur, &rotZCur);

    //FindRotation(rel_time, &rotXCur, &rotYCur, &rotZCur);

    float posXCur, posYCur, posZCur;
    posXCur = 0;
    posYCur = 0;
    posZCur = 0;
    FindPosition(rel_time, &posXCur, &posYCur, &posZCur);
    if (firstPointFlag == true)
    {
      LOG(INFO)<< "firstPointFlag"<<firstPointFlag<<std::endl;
      transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
      firstPointFlag = false;
    }

    // transform points to start
    Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
    Eigen::Affine3f transBt = transStartInverse * transFinal;

    PointT newPoint = point;
    newPoint.x = transBt(0, 0) * point.x + transBt(0, 1) * point.y + transBt(0, 2) * point.z + transBt(0, 3);
    newPoint.y = transBt(1, 0) * point.x + transBt(1, 1) * point.y + transBt(1, 2) * point.z + transBt(1, 3);
    newPoint.z = transBt(2, 0) * point.x + transBt(2, 1) * point.y + transBt(2, 2) * point.z + transBt(2, 3);
    newPoint.intensity = point.intensity;
    point = newPoint;
    //return newPoint;
  }

  void PointProcessor::PointToRing(const PointCloudConstPtr &cloud_in,
                                   vector<PointCloudPtr> &ring_out,
                                   vector<PointCloudPtr> &intensity_out)
  {
    //  const std::vector<PointT, Eigen::aligned_allocator<PointT> > &points = cloud_in->points;
    //  vector<int> indices;
    //  PointCloud cloud_filtered;
    //  pcl::removeNaNFromPointCloud(*cloud_in, cloud_filtered, indices);

    auto &points = cloud_in->points;
    size_t cloud_size = points.size();

    tic_toc_.Tic();

    ///>
    float startOri = -atan2(points[0].y, points[0].x);
    float endOri = -atan2(points[cloud_size - 1].y,
                          points[cloud_size - 1].x) +
                   2 * M_PI;

    if (endOri - startOri > 3 * M_PI)
    {
      endOri -= 2 * M_PI;
    }
    else if (endOri - startOri < M_PI)
    {
      endOri += 2 * M_PI;
    }
    bool halfPassed = false;
    int count = cloud_size;
    bool start_flag = false;
    ///<

    for (size_t i = 0; i < cloud_size; ++i)
    {
      PointT p = points[i];
      PointT p_with_intensity = points[i];

      // skip NaN and INF valued points
      if (!pcl_isfinite(p.x) ||
          !pcl_isfinite(p.y) ||
          !pcl_isfinite(p.z))
      {
        continue;
      }
  
      float dis = sqrt(p.x * p.x + p.y * p.y);
      float ele_rad = atan2(p.z, dis);
      float azi_rad = 2 * M_PI - atan2(p.y, p.x);

      ///>mine
#ifndef DEBUG_ORIGIN
      if (azi_rad >= 2 * M_PI)
      {
        azi_rad -= 2 * M_PI;
      }

      int scan_id = ElevationToRing(ele_rad);

      if (scan_id >= num_rings_ || scan_id < 0)
      {
        // DLOG(INFO) << RadToDeg(ele_rad) << ", " << scan_id;
        // DLOG(INFO) << (scan_id < 0 ? " point too low" : "point too high");
        continue;
      }

      if (!start_flag)
      {
        start_ori_ = azi_rad;
        start_flag = true;
      }

      // cout << scan_id << " " << azi_rad << endl;

      //    if (config_.deskew) {
      //      p.intensity = azi_rad;
      //    } else {
      //      p.intensity = scan_id;
      //    }

      p.intensity = azi_rad;
#endif
      ///<mine

      ///>origin
#ifdef DEBUG_ORIGIN
      int scan_id = ElevationToRing(ele_rad);

      if (scan_id >= num_rings_ || scan_id < 0)
      {
        // DLOG(INFO) << RadToDeg(ele_rad) << ", " << scan_id;
        // DLOG(INFO) << (scan_id < 0 ? " point too low" : "point too high");
        continue;
      }

      float ori = -atan2(p.y, p.x);
      if (!halfPassed)
      {
        if (ori < startOri - M_PI / 2)
        {
          ori += 2 * M_PI;
        }
        else if (ori > startOri + M_PI * 3 / 2)
        {
          ori -= 2 * M_PI;
        }

        if (ori - startOri > M_PI)
        {
          halfPassed = true;
        }
      }
      else
      {
        ori += 2 * M_PI;

        if (ori < endOri - M_PI * 3 / 2)
        {
          ori += 2 * M_PI;
        }
        else if (ori > endOri + M_PI / 2)
        {
          ori -= 2 * M_PI;
        }
      }

      float relTime = (ori - startOri) / (endOri - startOri);
#ifndef DEBUG
      p.intensity = scan_id + config_.scan_period * relTime;
#else
      p.intensity = config_.scan_period * relTime;
#endif
#endif
      ///<origin

      ring_out[scan_id]->push_back(p);
      intensity_out[scan_id]->push_back(p_with_intensity);
    }

    ROS_DEBUG_STREAM("ring time: " << tic_toc_.Toc() << " ms");

    tic_toc_.Tic();
    // TODO: De-skew with rel_time, this for loop is not necessary
    // start_ori_ = NAN;

    //  for (int ring = 0; ring < num_rings_; ++ring) {
    //    if (ring_out[ring]->size() <= 0) {
    //      continue;
    //    }
    //
    //    float azi_rad = ring_out[ring]->front().intensity;
    //    if (start_ori_ != start_ori_) {
    //      start_ori_ = azi_rad;
    //    } else {
    //      start_ori_ = (RadLt(start_ori_, azi_rad) ? start_ori_ : azi_rad);
    //    }
    //    // cout << azi_rad << endl;
    //  }
    // start_ori_ = ring_out[0]->front().intensity;

    // infer right start_ori_
    if (config_.infer_start_ori_)
    {
      std_msgs::Float32 start_ori_msg, start_ori_inferred_msg;
      start_ori_msg.data = start_ori_;

      start_ori_buf2_.push(start_ori_);
      if (start_ori_buf1_.size() >= 10)
      {
        float start_ori_diff1 = start_ori_buf1_.last() - start_ori_buf1_.first();
        float start_ori_step1 = NormalizeRad(start_ori_diff1) / 9;

        float start_ori_diff2 = start_ori_buf2_.last() - start_ori_buf2_.first();
        float start_ori_step2 = NormalizeRad(start_ori_diff2) / 9;

        DLOG(INFO) << "origin start_ori_: " << start_ori_;
        DLOG(INFO) << "start_ori_step1: " << start_ori_step1;
        DLOG(INFO) << "start_ori_step2: " << start_ori_step2;
        //       DLOG(INFO) << "diff: " << fabs(NormalizeRad(start_ori_ - start_ori_buf_.last()));
        if (fabs(NormalizeRad(start_ori_ - start_ori_buf1_.last())) > config_.rad_diff)
        {
          start_ori_ = start_ori_buf1_.last() + start_ori_step1;
          start_ori_ = NormalizeRad(start_ori_);
          if (start_ori_ < 0)
          {
            start_ori_ += 2 * M_PI;
          }
        }
        if (AbsRadDistance(start_ori_step1, start_ori_step2) < 0.05 && AbsRadDistance(start_ori_buf2_[9] - start_ori_buf2_[8], start_ori_step1) < 0.05 && AbsRadDistance(start_ori_buf2_[8] - start_ori_buf2_[7], start_ori_step1) < 0.05 && AbsRadDistance(start_ori_buf2_[7] - start_ori_buf2_[6], start_ori_step1) < 0.05 && AbsRadDistance(start_ori_buf2_[6] - start_ori_buf2_[5], start_ori_step1) < 0.05 && AbsRadDistance(start_ori_buf2_[5] - start_ori_buf2_[4], start_ori_step1) < 0.05 && AbsRadDistance(start_ori_buf2_[4] - start_ori_buf2_[3], start_ori_step1) < 0.05 && AbsRadDistance(start_ori_buf2_[3] - start_ori_buf2_[2], start_ori_step1) < 0.05 && AbsRadDistance(start_ori_buf2_[2] - start_ori_buf2_[1], start_ori_step1) < 0.05 && AbsRadDistance(start_ori_buf2_[1] - start_ori_buf2_[0], start_ori_step1) < 0.05)
        {
          start_ori_ = ring_out[0]->front().intensity;
        }
      }
      start_ori_buf1_.push(start_ori_);

      start_ori_inferred_msg.data = start_ori_;
      pub_start_ori_.publish(start_ori_msg);
      pub_start_ori_inferred_.publish(start_ori_inferred_msg);
    } // if

    //DLOG(INFO) << "start_ori: " << start_ori_;

    for (int ring = 0; ring < num_rings_; ++ring)
    {
      // points in a ring
      PointCloud &points_in_ring = (*ring_out[ring]);
      PointCloud &points_in_ring_with_intensity = (*intensity_out[ring]);
      size_t cloud_in_ring_size = points_in_ring.size();

      for (int i = 0; i < cloud_in_ring_size; ++i)
      {
        PointT &p = points_in_ring[i];
        PointT &p_with_intensity = points_in_ring_with_intensity[i];

        float azi_rad_rel = p.intensity - start_ori_;
        if (azi_rad_rel < 0)
        {
          azi_rad_rel += 2 * M_PI;
        }

        float rel_time = config_.scan_period * azi_rad_rel / (2 * M_PI);

        ///>mine
#ifndef DEBUG_ORIGIN

#ifndef DEBUG
        p.intensity = ring + rel_time;
        p_with_intensity.intensity = int(p_with_intensity.intensity) + rel_time;
#else
        p.intensity = rel_time;
#endif

#endif
        ///<mine
      } // for i
    }   // for ring
    ROS_DEBUG_STREAM("reorder time: " << tic_toc_.Toc() << " ms");
    PublishCloudMsg(pub_deskew_cloud_, *cloud_in, sweep_start_, "velodyne");

  }

  void PointProcessor::PointToRing(const pcl::PointCloud<PointIRT>::Ptr &cloud_in,
                                   vector<PointCloudPtr> &ring_out,
                                   vector<PointCloudPtr> &intensity_out)
  {
    auto &points = cloud_in->points;
    size_t cloud_size = points.size();

    tic_toc_.Tic();

    ///>
    int count = cloud_size;
    bool start_flag = false;
    start_ori_ = 0;
    end_ori_ = 0;
    bool half_passed = false;
    ///<
    pcl::PointCloud<PointIRT> cloud_deskew = *cloud_in;
    for (size_t i = 0; i < cloud_size; ++i)
    {
      auto &&p_ir = points[i];
      PointT p;
      p.x = p_ir.x;
      p.y = p_ir.y;
      p.z = p_ir.z;
      p.intensity = p_ir.intensity;
      //p.time = p_ir.time;

      PointT p_with_intensity = p;

      // skip NaN and INF valued points
      if (!pcl_isfinite(p.x) ||
          !pcl_isfinite(p.y) ||
          !pcl_isfinite(p.z))
      {
        continue;
      }
      if (deskew_flag)
      {
        //tod
        // LOG(INFO)<<"p time:"<< p_ir.time<<std::endl;
        DeSkew(p, p_ir.time);
        cloud_deskew.points[i].x = p.x ;
        cloud_deskew.points[i].y = p.y ;
        cloud_deskew.points[i].z = p.z ;
        cloud_deskew.points[i].intensity = p.intensity ;

        // DeSkew(p_with_intensity, p_with_intensity.time);
      }
      float dis = sqrt(p.x * p.x + p.y * p.y);
      float ele_rad = atan2(p.z, dis);
      float azi_rad = 2 * M_PI - atan2(p.y, p.x);

      if (azi_rad >= 2 * M_PI)
      {
        azi_rad -= 2 * M_PI;
      }

      int scan_id = p_ir.ring;

      if (scan_id >= num_rings_ || scan_id < 0)
      {
        // DLOG(INFO) << RadToDeg(ele_rad) << ", " << scan_id;
        // DLOG(INFO) << (scan_id < 0 ? " point too low" : "point too high");
        continue;
      }

      if (!start_flag)
      {
        start_ori_ = azi_rad;
        start_flag = true;
      }

      float azi_rad_rel = azi_rad - start_ori_;
      float azi_rad_bak = azi_rad;

      if (!half_passed)
      {
        if (azi_rad_rel < 0)
        {
          azi_rad += 2 * M_PI;
        }
        if (azi_rad_rel > M_PI && i > 3 * cloud_size / 2)
        {
          half_passed = true;
        }
      }
      else
      {
        if (azi_rad_rel < M_PI / 2)
        {
          azi_rad += 2 * M_PI;
        }
      }

      if (end_ori_ < azi_rad)
      {
        end_ori_ = azi_rad;
      }
      p.intensity = azi_rad;
      PointT tmp_p, tmp_p_with_intensity;
      tmp_p.x = p.x;
      tmp_p.y = p.y;
      tmp_p.z = p.z;
      tmp_p.intensity = p.intensity;

      tmp_p_with_intensity.x = p_with_intensity.x;
      tmp_p_with_intensity.y = p_with_intensity.y;
      tmp_p_with_intensity.z = p_with_intensity.z;
      tmp_p_with_intensity.intensity = p_with_intensity.intensity;
      ring_out[scan_id]->push_back(tmp_p);

      intensity_out[scan_id]->push_back(tmp_p_with_intensity);
    }

    ROS_DEBUG_STREAM("ring time: " << tic_toc_.Toc() << " ms");

    tic_toc_.Tic();
    // TODO: De-skew with rel_time, this for loop is not necessary

    float range_ori = end_ori_ - start_ori_;
    ROS_DEBUG_STREAM("start_ori_: " << start_ori_ << " end_ori_: " << end_ori_
                                    << " range_ori: " << range_ori);

    for (int ring = 0; ring < num_rings_; ++ring)
    {
      // points in a ring
      PointCloud &points_in_ring = (*ring_out[ring]);
      PointCloud &points_in_ring_with_intensity = (*intensity_out[ring]);
      size_t cloud_in_ring_size = points_in_ring.size();

      for (int i = 0; i < cloud_in_ring_size; ++i)
      {
        PointT &p = points_in_ring[i];
        PointT &p_with_intensity = points_in_ring_with_intensity[i];

        float azi_rad_rel = p.intensity - start_ori_;

        float rel_time = config_.scan_period * azi_rad_rel / range_ori;

        // TODO: consider encode rel_time from packets
        p.intensity = ring + rel_time;
        p_with_intensity.intensity = int(p_with_intensity.intensity) + rel_time;
      }
    }
    ROS_DEBUG_STREAM("reorder time: " << tic_toc_.Toc() << " ms");
    PublishCloudMsg(pub_deskew_cloud_, cloud_deskew, sweep_start_, "velodyne");

  }

  void PointProcessor::SetDeskew(bool deskew)
  {
    config_.deskew = deskew;
  }

  void PointProcessor::PrepareRing(const PointCloud &scan)
  {
    // const PointCloud &scan = laser_scans[idx_ring];
    size_t scan_size = scan.size();
    scan_ring_mask_.resize(scan_size);
    scan_ring_mask_.assign(scan_size, 0);
    for (size_t i = 0 + config_.num_curvature_regions; i < scan_size - config_.num_curvature_regions; ++i)
    {
      const PointT &p_prev = scan[i - 1];
      const PointT &p_curr = scan[i];
      const PointT &p_next = scan[i + 1];

      float diff_next2 = CalcSquaredDiff(p_curr, p_next);

      // about 30 cm
      if (diff_next2 > 0.1)
      {
        float depth = CalcPointDistance(p_curr);
        float depth_next = CalcPointDistance(p_next);

        if (depth > depth_next)
        {
          // to closer point
          float weighted_diff = sqrt(CalcSquaredDiff(p_next, p_curr, depth_next / depth)) / depth_next;
          // relative distance
          if (weighted_diff < 0.1)
          {
            fill_n(&scan_ring_mask_[i - 0 - config_.num_curvature_regions], config_.num_curvature_regions + 1, 1);
            continue;
          }
        }
        else
        {
          float weighted_diff = sqrt(CalcSquaredDiff(p_curr, p_next, depth / depth_next)) / depth;
          if (weighted_diff < 0.1)
          {
            fill_n(&scan_ring_mask_[i - 0 + 1], config_.num_curvature_regions + 1, 1);
            continue;
          }
        }
      }

      float diff_prev2 = CalcSquaredDiff(p_curr, p_prev);
      float dis2 = CalcSquaredPointDistance(p_curr);

      // for this point -- 1m -- 1.5cm
      if (diff_next2 > 0.0002 * dis2 && diff_prev2 > 0.0002 * dis2)
      {
        scan_ring_mask_[i - 0] = 1;
      }
    }
  }

  void PointProcessor::PrepareSubregion(const PointCloud &scan, const size_t idx_start, const size_t idx_end)
  {

    //  cout << ">>>>>>> " << idx_ring << ", " << idx_start << ", " << idx_end << " <<<<<<<" << endl;
    //  const PointCloud &scan = laser_scans[idx_ring];
    size_t region_size = idx_end - idx_start + 1;
    curvature_idx_pairs_.resize(region_size);
    subregion_labels_.resize(region_size);
    subregion_labels_.assign(region_size, SURFACE_LESS_FLAT);

    int num_point_neighbors = 2 * config_.num_curvature_regions;

    for (size_t i = idx_start, in_region_idx = 0; i <= idx_end; ++i, ++in_region_idx)
    {
      float diff_x = -num_point_neighbors * scan[i].x;
      float diff_y = -num_point_neighbors * scan[i].y;
      float diff_z = -num_point_neighbors * scan[i].z;

      for (int j = 1; j <= config_.num_curvature_regions; j++)
      {
        diff_x += scan[i + j].x + scan[i - j].x;
        diff_y += scan[i + j].y + scan[i - j].y;
        diff_z += scan[i + j].z + scan[i - j].z;
      }

      float curvature = diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;
      pair<float, size_t> curvature_idx_(curvature, i);
      curvature_idx_pairs_[in_region_idx] = curvature_idx_;
      //    _regionCurvature[regionIdx] = diffX * diffX + diffY * diffY + diffZ * diffZ;
      //    _regionSortIndices[regionIdx] = i;
    }

    sort(curvature_idx_pairs_.begin(), curvature_idx_pairs_.end());

    //  for (const auto &pair : curvature_idx_pairs_) {
    //    cout << pair.first << " " << pair.second << endl;
    //  }
  }

  void PointProcessor::MaskPickedInRing(const PointCloud &scan, const size_t in_scan_idx)
  {

    // const PointCloud &scan = laser_scans[idx_ring];
    scan_ring_mask_[in_scan_idx] = 1;

    for (int i = 1; i <= config_.num_curvature_regions; ++i)
    {
      /// 20cm
      if (CalcSquaredDiff(scan[in_scan_idx + i], scan[in_scan_idx + i - 1]) > 0.05)
      {
        break;
      }

      scan_ring_mask_[in_scan_idx + i] = 1;
    }

    for (int i = 1; i <= config_.num_curvature_regions; ++i)
    {
      if (CalcSquaredDiff(scan[in_scan_idx - i], scan[in_scan_idx - i + 1]) > 0.05)
      {
        break;
      }

      scan_ring_mask_[in_scan_idx - i] = 1;
    }
  }

  void PointProcessor::ExtractFeaturePoints()
  {

    tic_toc_.Tic();

    ///< i is #ring, j is #subregion, k is # in region
    for (size_t i = 0; i < num_rings_; ++i)
    {

      PointCloud::Ptr surf_points_less_flat_ptr(new PointCloud());

      size_t start_idx = scan_ranges[i].first;
      size_t end_idx = scan_ranges[i].second;

      // skip too short scans
      if (end_idx <= start_idx + 2 * config_.num_curvature_regions)
      {
        continue;
      }

      const PointCloud &scan_ring = *laser_scans[i];
      size_t scan_size = scan_ring.size();

      PrepareRing(scan_ring);

      // extract features from equally sized scan regions
      for (int j = 0; j < config_.num_scan_subregions; ++j)
      {
        // ((s+d)*N+j*(e-s-2d))/N, ((s+d)*N+(j+1)*(e-s-2d))/N-1
        size_t sp = ((0 + config_.num_curvature_regions) * (config_.num_scan_subregions - j) + (scan_size - config_.num_curvature_regions) * j) / config_.num_scan_subregions;
        size_t ep = ((0 + config_.num_curvature_regions) * (config_.num_scan_subregions - 1 - j) + (scan_size - config_.num_curvature_regions) * (j + 1)) / config_.num_scan_subregions - 1;

        // skip empty regions
        if (ep <= sp)
        {
          continue;
        }

        size_t region_size = ep - sp + 1;
        PrepareSubregion(scan_ring, sp, ep);

        // extract corner features
        int num_largest_picked = 0;
        for (size_t k = region_size; k > 0 && num_largest_picked < config_.max_corner_less_sharp;)
        {
          // k must be greater than 0
          const pair<float, size_t> &curvature_idx = curvature_idx_pairs_[--k];
          float curvature = curvature_idx.first;
          size_t idx = curvature_idx.second;
          size_t in_scan_idx = idx - 0; // scan start index is 0 for all ring scans
          size_t in_region_idx = idx - sp;

          if (scan_ring_mask_[in_scan_idx] == 0 && curvature > config_.surf_curv_th)
          {
            ++num_largest_picked;
            if (num_largest_picked <= config_.max_corner_sharp)
            {
              subregion_labels_[in_region_idx] = CORNER_SHARP;
              corner_points_sharp_.push_back(scan_ring[in_scan_idx]);
            }
            else
            {
              subregion_labels_[in_region_idx] = CORNER_LESS_SHARP;
            }
            corner_points_less_sharp_.push_back(scan_ring[in_scan_idx]);

            MaskPickedInRing(scan_ring, in_scan_idx);
          }
        }

        // extract flat surface features
        int num_smallest_picked = 0;
        for (int k = 0; k < region_size && num_smallest_picked < config_.max_surf_flat; ++k)
        {
          const pair<float, size_t> &curvature_idx = curvature_idx_pairs_[k];
          float curvature = curvature_idx.first;
          size_t idx = curvature_idx.second;
          size_t in_scan_idx = idx - 0; // scan start index is 0 for all ring scans
          size_t in_region_idx = idx - sp;

          if (scan_ring_mask_[in_scan_idx] == 0 && curvature < config_.surf_curv_th)
          {
            ++num_smallest_picked;
            subregion_labels_[in_region_idx] = SURFACE_FLAT;
            surface_points_flat_.push_back(scan_ring[in_scan_idx]);

            MaskPickedInRing(scan_ring, in_scan_idx);
          }
        }

        // extract less flat surface features
        for (int k = 0; k < region_size; ++k)
        {
          if (subregion_labels_[k] <= SURFACE_LESS_FLAT)
          {
            surf_points_less_flat_ptr->push_back(scan_ring[sp + k]);
          }
        }

      } /// j

      // down size less flat surface point cloud of current scan
      PointCloud surf_points_less_flat_downsampled;
      pcl::VoxelGrid<PointT> down_size_filter;
      down_size_filter.setInputCloud(surf_points_less_flat_ptr);
      down_size_filter.setLeafSize(config_.less_flat_filter_size,
                                   config_.less_flat_filter_size,
                                   config_.less_flat_filter_size);

      if (surf_points_less_flat_ptr->empty())
      {
        continue;
      }
      else
      {

        down_size_filter.filter(surf_points_less_flat_downsampled);
        surface_points_less_flat_ += surf_points_less_flat_downsampled;
      }

    } /// i

    size_t less_flat_cloud_size = surface_points_less_flat_.size();

    for (int i = 0; i < less_flat_cloud_size; ++i)
    {
      PointT &p = surface_points_less_flat_[i];

      float azi_rad = 2 * M_PI - atan2(p.y, p.x);

      if (azi_rad >= 2 * M_PI)
      {
        azi_rad -= 2 * M_PI;
      }

      float azi_rad_rel = azi_rad - start_ori_;
      if (azi_rad_rel < 0)
      {
        azi_rad_rel += 2 * M_PI;
      }

      float rel_time = config_.scan_period * azi_rad_rel / (2 * M_PI);

      if (rel_time > config_.scan_period || rel_time < 0.0)
      {
        LOG(ERROR) << "config_.scan_period: " << config_.scan_period;
        LOG(ERROR) << "rel_time error: " << rel_time;
      }
      p.intensity = int(p.intensity) + rel_time;
    }

    //ROS_DEBUG_STREAM("extract features time: " << tic_toc_.Toc() << " ms");
    //DLOG(INFO) << "extract features time: " << tic_toc_.Toc() << " ms";

  } // ExtractFeaturePoints

  void PointProcessor::PublishResults()
  {

    if (!is_ros_setup_)
    {
      DLOG(WARNING) << "ros is not set up, and no results will be published";
      return;
    }
    // publish full resolution and feature point clouds
    PublishCloudMsg(pub_full_cloud_, cloud_in_rings_, sweep_start_, config_.capture_frame_id);
    PublishCloudMsg(pub_corner_points_sharp_, corner_points_sharp_, sweep_start_, config_.capture_frame_id);
    PublishCloudMsg(pub_corner_points_less_sharp_, corner_points_less_sharp_, sweep_start_, config_.capture_frame_id);
    PublishCloudMsg(pub_surf_points_flat_, surface_points_flat_, sweep_start_, config_.capture_frame_id);
    PublishCloudMsg(pub_surf_points_less_flat_, surface_points_less_flat_, sweep_start_, config_.capture_frame_id);
  }

} // namespace lclio