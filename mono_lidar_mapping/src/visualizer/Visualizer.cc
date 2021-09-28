#include "visualizer/Visualizer.h"

Visualizer::Visualizer(camodocal::CameraPtr cam):cam_(cam),depth_map_util{}
{
    nh_ = ros::NodeHandle("fused");
    pub_depth_map_ = nh_.advertise<sensor_msgs::Image>("img_depth",1000);
    pub_img_track_ = nh_.advertise<sensor_msgs::Image>("img_track",1000);
    pub_projection_ = nh_.advertise<sensor_msgs::Image>("img_projection",1000);

    pub_original_odom_ = nh_.advertise<nav_msgs::Odometry>("origin_odometry",1000);
    pub_new_odom_ = nh_.advertise<nav_msgs::Odometry>("new_odometry",1000);
    pub_new_camera_odom_ = nh_.advertise<nav_msgs::Odometry>("new_camera_odometry",1000);
    pub_extrinsic_ = nh_.advertise<nav_msgs::Odometry>("extrinsic",1000);
    pub_rgb_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("color_cloud",1000);
    pub_keyframe_point_ = nh_.advertise<sensor_msgs::PointCloud>("keyframe_point", 1000);
}

Visualizer::~Visualizer(){}

void Visualizer::pubTrackImg(const cv::Mat &imgTrack, double t)
{
    std_msgs::Header header;
    header.frame_id = "camera";
    header.stamp = ros::Time(t);

    cv_bridge::CvImage img_track_msg;
    img_track_msg.header = header;
    img_track_msg.encoding = sensor_msgs::image_encodings::BGR8;
    img_track_msg.image = imgTrack.clone();
    pub_img_track_.publish(img_track_msg);
}

void Visualizer::pubOriginalOdom(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, double t)
{
    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time(t);
    odometry.header.frame_id = "laser_init";
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();

    odometry.pose.pose.orientation.w = Q.w();
    odometry.pose.pose.orientation.x = Q.x();
    odometry.pose.pose.orientation.y = Q.y();
    odometry.pose.pose.orientation.z = Q.z();

    pub_original_odom_.publish(odometry);

}

void Visualizer::pubNewOdom(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, double t)
{
    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time(t);
    odometry.header.frame_id = "camera_init";
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();

    odometry.pose.pose.orientation.w = Q.w();
    odometry.pose.pose.orientation.x = Q.x();
    odometry.pose.pose.orientation.y = Q.y();
    odometry.pose.pose.orientation.z = Q.z();

    std::cout << "position: " << P.transpose() << std::endl;
    std::cout <<"orientation: \n" << Q.toRotationMatrix() << std::endl;

    pub_new_odom_.publish(odometry);
}

void Visualizer::pubCamNewOdom(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, double t)
{
    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time(t);
    odometry.header.frame_id = "camera_init";
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();

    odometry.pose.pose.orientation.w = Q.w();
    odometry.pose.pose.orientation.x = Q.x();
    odometry.pose.pose.orientation.y = Q.y();
    odometry.pose.pose.orientation.z = Q.z();

    std::cout << "cam position: " << P.transpose() << std::endl;
    std::cout <<"cam orientation: \n" << Q.toRotationMatrix() << std::endl;

    pub_new_camera_odom_.publish(odometry);
}

void Visualizer::pubExtrinsic(const Eigen::Vector3d &P, const Eigen::Matrix3d &R, double t)
{   
    Eigen::Quaterniond Q(R);
    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time(t);
    odometry.header.frame_id = "camera_init";

    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();
    
    odometry.pose.pose.orientation.x = Q.x();
    odometry.pose.pose.orientation.y = Q.y();
    odometry.pose.pose.orientation.z = Q.z();
    odometry.pose.pose.orientation.w = Q.w();
    pub_extrinsic_.publish(odometry);
}

void Visualizer::pubKeyframePoints(const sensor_msgs::PointCloud &point_cloud)
{
    // for(size_t i = 0; i < point_cloud.points.size(); i++)
    // {
    //     printf("point id %d, 3D point: %f, %f, %f, 2D nromal point: %f, %f, 2D uv point: %f, %f\n",
    //             (int)point_cloud.channels[i].values[4], point_cloud.points[i].x, point_cloud.points[i].y, point_cloud.points[i].z,
    //                 point_cloud.channels[i].values[0], point_cloud.channels[i].values[1],
    //                 point_cloud.channels[i].values[2], point_cloud.channels[i].values[3]);
    // }

    pub_keyframe_point_.publish(point_cloud);
}

void Visualizer::pubNewCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud, double t)
{
    sensor_msgs::PointCloud2 point_msg;

    pcl::toROSMsg(*rgb_cloud,point_msg);

    point_msg.header.frame_id = "camera";
    point_msg.header.stamp = ros::Time(t);
    
    pub_rgb_cloud_.publish(point_msg);
}

cv::Point2f Visualizer::Point3DTo2D(const pcl::PointXYZ &pt)
{
    Eigen::Vector3d P;
    P.x() = pt.x;
    P.y() = pt.y;
    P.z() = pt.z;

    Eigen::Vector2d p_dst;
    cam_->spaceToPlane(P,p_dst);

    return cv::Point2f(p_dst.x(), p_dst.y());
}

