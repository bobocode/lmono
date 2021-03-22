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
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <thread>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <geometry_msgs/Quaternion.h>

#include "parameter.h"
#include "image_process/Estimator.h"
#include "laser_odometry/PointOdometry.h"
#include "utils/TicToc.h"

using namespace std;
static ros::NodeHandlePtr nh_ptr;

std::mutex loop_mutex_;

std::queue<sensor_msgs::PointCloudConstPtr> loop_buf;

void loopHandler(const sensor_msgs::PointCloudConstPtr &point_msg)
{
    loop_mutex_.lock();
    loop_buf.push(point_msg);
    loop_mutex_.unlock();
}

void Run()
{
    Estimator estimator;

    readParameters(*nh_ptr);
    estimator.setParameter();
    estimator.setupRos(*nh_ptr);

    PointOdometry odometry(0.1, ODOM_IO);
    odometry.SetupRos(*nh_ptr);
    odometry.Reset();

    thread odom(&PointOdometry::Spin, &odometry);
    thread measurement_manager(&Estimator::measurementHandler, &estimator);
    thread loop_manager(&Estimator::setLoopFrame,&estimator);
    //thread measurement_process(&Estimator::processEstimation, &estimator);

   // boost::thread visualizer(boost::bind(&CloudVisualizer::Spin, &(estimator.cloud_vis)));
    
    //std::thread loop_measurment{process};
    ros::spin();
    // ros::Rate r(1000);
    // while(ros::ok())
    // {
    //     ros::spinOnce();
    //     r.sleep();
    // }

}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"auto_calib");
    
    ros::NodeHandle nh("~");
    nh_ptr = boost::make_shared<ros::NodeHandle>(nh);
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    /*if(argc != 2)
    {
        printf("please intput: rosrun auto_calib calib_node [config file] \n"
               "for example: rosrun auto_calib calib_node "
               "~/ \n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);*/

    Run();
    ROS_WARN("waiting for images and  compact");
    

    return 0;
}