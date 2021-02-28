
#include "image_process/Measurement_Manager.h"

void MeasurementManager::setupRos(ros::NodeHandle &nh)
{
    sub_image_ = nh.subscribe(IMAGE0_TOPIC, 10, &MeasurementManager::ImageHandler, this);
    sub_compact_data_ = nh.subscribe(POINTS_TOPIC,10, &MeasurementManager::CompactDataHandler,this);
    sub_laser_odom_ = nh.subscribe(LASER_ODOM_TOPIC, 10, &MeasurementManager::LaserOdomHandler,this);
}

PairMeasurements MeasurementManager::GetMeasurements()
{
    PairMeasurements measurements;
    while(1)
    {
        if(!img0_buf.empty() && !laser_buf.empty())
        {
            if(img0_buf.front()->header.stamp.toSec() < laser_buf.front()->header.stamp.toSec() - 0.003)
            {
                img0_buf.pop();
            }else if(img0_buf.front()->header.stamp.toSec() > laser_buf.front()->header.stamp.toSec() + 0.003)
            {
                laser_buf.pop();
            }else
            {
                sensor_msgs::PointCloud2ConstPtr point_cloud_msg = laser_buf.front();
                laser_buf.pop();

                sensor_msgs::ImageConstPtr img_msg = img0_buf.front();
                img0_buf.pop();

                measurements.emplace_back(img_msg, point_cloud_msg);
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
    buf_mutex_.lock();
    laser_buf.push(pts_msg);
    buf_mutex_.unlock();
    con_.notify_one();

}

void MeasurementManager::ImageHandler(const sensor_msgs::ImageConstPtr &img0_msg)
{
    buf_mutex_.lock();
    img0_buf.push(img0_msg);
    buf_mutex_.unlock();
    con_.notify_one();

}


