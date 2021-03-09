
#include <gtest/gtest.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Quaternion.h>

#include "laser_odometry/PointMapping.h"
#include "utils/TicToc.h"

using namespace lclio;
using namespace std;
using namespace mathutils;

int main(int argc, char **argv) {

    //FLAGS_log_dir = "/home/zjc/LIO_Test_new/LIO_test_ws/src/log";
    google::InitGoogleLogging(argv[0]);
    //FLAGS_alsologtostderr = true;

    ros::init(argc, argv, "point_mapping");

    ros::NodeHandle nh("~");;

    PointMapping mapper;
    mapper.SetupRos(nh);
    mapper.Reset();

    ros::Rate r(100);
    while (ros::ok()) {
      mapper.Process();
      ros::spinOnce();
      r.sleep();
    }

    return 0;
}