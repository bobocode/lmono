#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "parameter.h"
#include "image_process/Estimator.h"

using namespace std;
ros::NodeHandle nh("~");

void Run()
{
    Estimator estimator;

    readParameters(nh);
    estimator.setParameter();
    estimator.setupRos(nh);

    PointOdometry odometry(0.1, ODOM_IO);
    odometry.SetupRos(nh);
    odometry.Reset();

    thread odom(&PointOdometry::Spin, &odometry);
    thread measurement_manager(&Estimator::processEstimation, &estimator);

    ros::Rate r(1000);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"auto_calib");
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
    ROS_WARN("waiting for images and lidar odom");
    

    return 0;
}