
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
    sub_image_ = nh.subscribe(IMAGE0_TOPIC, 500, &MeasurementManager::ImageHandler, this);
    //sub_compact_data_ = nh.subscribe(POINTS_TOPIC,500, &MeasurementManager::CompactDatahandler,this);
    sub_matched_points_ = nh.subscribe("/monolio_loop_detection/matched_points",2000, &MeasurementManager::LoopHandler,this);
    //sub_original_points_ = nh.subscribe(POINTS_TOPIC, 10, &MeasurementManager::PointCloudHandler,this);
    sub_laser_odom_ = nh.subscribe(LASER_ODOM_TOPIC, 10, &MeasurementManager::LaserOdomHandler,this);
}

/*PairMeasurements MeasurementManager::GetMeasurements()
{
    PairMeasurements measurements;
    while(1)
    {
        if(img0_buf.empty() || compact_data_buf.empty())
        {
            return measurements;
        }

        if(img0_buf.front()->header.stamp.toSec() < compact_data_buf.front()->header.stamp.toSec() - DELAY_TIME)
        {
            double delta_t_ = compact_data_buf.front()->header.stamp.toSec() - img0_buf.front()->header.stamp.toSec();
            //printf("throwing image %f, %f\n", img0_buf.front()->header.stamp.toSec(), delta_t_);

            img0_buf.pop();
        }else if(img0_buf.front()->header.stamp.toSec() > compact_data_buf.front()->header.stamp.toSec() + DELAY_TIME)
        {
            double delta_t_ = img0_buf.front()->header.stamp.toSec() - compact_data_buf.front()->header.stamp.toSec();
            //printf("throwing compact date %f, %f\n", compact_data_buf.front()->header.stamp.toSec(),delta_t_);
            compact_data_buf.pop();
        }else
        {
            sensor_msgs::PointCloud2ConstPtr compact_data_msg = compact_data_buf.front();
            compact_data_buf.pop();

            sensor_msgs::ImageConstPtr img_msg = img0_buf.front();
            img0_buf.pop();

            // printf("measure img with stamp %f\n", img_msg->header.stamp.toNSec());
            // printf("measure laser with stamp %f\n", compact_data_msg->header.stamp.toNSec());

            printf("mearure img with stamp %f\n" , img_msg->header.stamp.toSec());
            printf("measure laser with stamp %f\n" ,compact_data_msg->header.stamp.toSec());

            measurements.emplace_back(img_msg, compact_data_msg);
            return measurements;
        }
    }

    return measurements;
}*/

PairMeasurements MeasurementManager::GetMeasurements()
{
    PairMeasurements measurements;
    while(1)
    {
        if(img0_buf.empty() || laser_odometry_buf.empty())
        {
            return measurements;
        }

        if(img0_buf.front()->header.stamp.toSec() < laser_odometry_buf.front()->header.stamp.toSec() - DELAY_TIME)
        {
            double delta_t_ = laser_odometry_buf.front()->header.stamp.toSec() - img0_buf.front()->header.stamp.toSec();
            //printf("throwing image %f, %f\n", img0_buf.front()->header.stamp.toSec(), delta_t_);

            img0_buf.pop();
        }else if(img0_buf.front()->header.stamp.toSec() > laser_odometry_buf.front()->header.stamp.toSec() + DELAY_TIME)
        {
            double delta_t_ = img0_buf.front()->header.stamp.toSec() - laser_odometry_buf.front()->header.stamp.toSec();
            //printf("throwing compact date %f, %f\n", compact_data_buf.front()->header.stamp.toSec(),delta_t_);
            laser_odometry_buf.pop();
        }else
        {
            nav_msgs::Odometry::ConstPtr odometry_msg = laser_odometry_buf.front();
            laser_odometry_buf.pop();

            sensor_msgs::ImageConstPtr img_msg = img0_buf.front();
            img0_buf.pop();

            // printf("measure img with stamp %f\n", img_msg->header.stamp.toNSec());
            // printf("measure laser with stamp %f\n", compact_data_msg->header.stamp.toNSec());

            printf("mearure img with stamp %f\n" , img_msg->header.stamp.toSec());
            printf("measure laser with stamp %f\n" ,odometry_msg->header.stamp.toSec());

            measurements.emplace_back(img_msg, odometry_msg);
            return measurements;
        }
    }

    return measurements;
}

vector<sensor_msgs::PointCloudConstPtr> MeasurementManager::LoopMeasurements()
{
   std::vector<sensor_msgs::PointCloudConstPtr> loop_measurements;
    while(true)
    {
        sensor_msgs::PointCloudConstPtr loop_msg = NULL;

        //printf(RED"check loop buf\n" WHT)

        if(!loop_buf.empty())
        {   
            loop_msg = loop_buf.front();
            loop_buf.pop();
            printf(RED"loop buf is not empty %f\n" WHT, loop_msg->header.stamp.toSec());

            if(loop_msg != NULL)
            {
                loop_measurements.push_back(loop_msg);
                return loop_measurements;
            }

        }else
        {
            return loop_measurements;
        }
    }

    return loop_measurements;

}

void MeasurementManager::LoopHandler(const sensor_msgs::PointCloudConstPtr &loop_msg)
{
    printf(RED"loop detection received %f\n"WHT, loop_msg->header.stamp.toSec());
    loop_mutex_.lock();
    loop_buf.push(loop_msg);
    loop_mutex_.unlock();
    con_.notify_one();
}

void MeasurementManager::LaserOdomHandler(const nav_msgs::Odometry::ConstPtr &laser_odom_msg)
{
    buf_mutex_.lock();
    printf("receiving laser odometry data %f\n",laser_odom_msg->header.stamp.toSec());
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
    //printf("receiving pts data %f\n",compact_data_msg->header.stamp.toSec());
    compact_data_buf.push(compact_data_msg);
    compact_buf_mutex_.unlock();
    con_.notify_one();
}

void MeasurementManager::ImageHandler(const sensor_msgs::ImageConstPtr &img0_msg)
{
    //printf("receiving img msg\n");
    img_buf_mutex_.lock();
    //printf("receiving img msg %f\n",img0_msg->header.stamp.toSec());
    img0_buf.push(img0_msg);
    img_buf_mutex_.unlock();
    con_.notify_one();
}


