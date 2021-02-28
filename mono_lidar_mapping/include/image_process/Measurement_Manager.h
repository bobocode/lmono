#ifndef _MEASUREMENTMANAGER_H_
#define _MEASUREMENTMANAGER_H_

#include <stdio.h>
#include <cstdlib>
#include <queue>
#include <vector>
#include <map>
#include <string>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <ros/ros.h>
#include <glog/logging.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "parameter.h"
#include "utils/CircularBuffer.h"
#include "utils/Twist.h"
#include "utils/common_ros.h"
#include "utils/TicToc.h"
#include "utils/math_utils.h"
#include "utils/geometry_utils.h"

using namespace std;
typedef std::pair<sensor_msgs::ImageConstPtr,sensor_msgs::PointCloud2ConstPtr> PairMeasurement;
typedef vector<PairMeasurement> PairMeasurements;

class MeasurementManager
{
    public:
        virtual void setupRos(ros::NodeHandle &nh);
        void LaserOdomHandler(const nav_msgs::Odometry::ConstPtr &laser_odom_msg);
        void PointCloudHandler(const sensor_msgs::PointCloud2ConstPtr &pts_msg);
        void ImagesHandler(const sensor_msgs::ImageConstPtr &img0_msg, const sensor_msgs::ImageConstPtr &img1_msg);
        void ImageHandler(const sensor_msgs::ImageConstPtr &img0_msg);

        void CompactDataHandler(const sensor_msgs::PointCloud2ConstPtr &compact_data_msg);
        PairMeasurements GetMeasurements();

    protected:
        ros::NodeHandle nh_;
        std::mutex buf_mutex_;
        std::mutex state_mutex_;
        std::mutex thread_mutex_;
        std::condition_variable con_;

        std::queue<sensor_msgs::ImageConstPtr> img0_buf;
        std::queue<sensor_msgs::ImageConstPtr> img1_buf;
        std::queue<nav_msgs::Odometry::ConstPtr> laser_odometry_buf;
        std::queue<sensor_msgs::PointCloud2ConstPtr> laser_buf;

        //message_filters::Subscriber<sensor_msgs::Image> sub_img0(nh,IMAGE0_TOPIC,100);
        //message_filters::Subscriber<sensor_msgs::Image> sub_img1(nh, IMAGE1_TOPIC,100);

        ros::Subscriber sub_image_;
        ros::Subscriber sub_laser_odom_;
        ros::Subscriber sub_compact_data_;

};

#endif
