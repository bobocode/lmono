
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
#include "image_process/Measurement_Manager.h"

void MeasurementManager::setupRos(ros::NodeHandle &nh)
{
    sub_image_ = nh.subscribe(IMAGE0_TOPIC, 5, &MeasurementManager::ImageHandler, this);
    sub_compact_data_ = nh.subscribe("/compact_data",5, &MeasurementManager::CompactDatahandler,this);
    //sub_original_points_ = nh.subscribe(POINTS_TOPIC, 10, &MeasurementManager::PointCloudHandler,this);
    
    //sub_laser_odom_ = nh.subscribe(LASER_ODOM_TOPIC, 10, &MeasurementManager::LaserOdomHandler,this);
}

PairMeasurements MeasurementManager::GetMeasurements()
{
    PairMeasurements measurements;
    while(1)
    {
        if(!img0_buf.empty() && !compact_data_buf.empty())
        {
            double time0 = img0_buf.front()->header.stamp.toSec();
            double time1 = compact_data_buf.front()->header.stamp.toSec();

            if(time0 < time1 - 0.003)
            {
                img0_buf.pop();
                return measurements;

            }else if(time0 > time1 + 0.003)
            {
                compact_data_buf.pop();
                return measurements;

            }else
            {   
                sensor_msgs::PointCloud2ConstPtr compact_data_msg = compact_data_buf.front();
                compact_data_buf.pop();

                sensor_msgs::ImageConstPtr img_msg = img0_buf.front();
                img0_buf.pop();

                //ROS_INFO_STREAM("mearure img with stamp " << time0);
                //ROS_INFO_STREAM("measure laser with stamp " << time1);

                measurements.emplace_back(img_msg, compact_data_msg);

                return measurements;
            }

        }else
        {
            return measurements;
        }
        
    }
}

void MeasurementManager::LaserOdomHandler(const nav_msgs::Odometry::ConstPtr &laser_odom_msg)
{
    buf_mutex_.lock();
    laser_odometry_buf.push(laser_odom_msg);
    buf_mutex_.unlock();
    con_.notify_one();
    
}

void MeasurementManager::PointCloudHandler(const sensor_msgs::PointCloud2ConstPtr &pts_msg)
{
    laser_buf_mutex_.lock();
    //ROS_INFO_STREAM("receiving pts data " << pts_msg->header.stamp.toSec());
    laser_data_buf.push(pts_msg);
    laser_buf_mutex_.unlock();
    con_.notify_one();

}

void MeasurementManager::CompactDatahandler(const sensor_msgs::PointCloud2ConstPtr &compact_data_msg)
{
    compact_buf_mutex_.lock();
    compact_data_buf.push(compact_data_msg);
    compact_buf_mutex_.unlock();
    con_.notify_one();
}

void MeasurementManager::ImageHandler(const sensor_msgs::ImageConstPtr &img0_msg)
{
    //printf("receiving img msg\n");
    img_buf_mutex_.lock();
    //ROS_INFO_STREAM("receiving img msg " << img0_msg->header.stamp.toSec());
    img0_buf.push(img0_msg);
    img_buf_mutex_.unlock();
    con_.notify_one();
}


