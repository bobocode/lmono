#include "visualizer/Visualizer.h"

double clip(double n, double lower, double upper)
{
    return std::max(lower, std::min(n, upper));
}

Visualizer::Visualizer(camodocal::CameraPtr cam):cam_(cam),depth_map_util{}
{
    nh_ = ros::NodeHandle("fused");
    pub_depth_map_ = nh_.advertise<sensor_msgs::Image>("img_depth",1000);
    pub_img_track_ = nh_.advertise<sensor_msgs::Image>("img_track",1000);
    pub_projection_ = nh_.advertise<sensor_msgs::Image>("img_projection",1000);

    pub_original_odom_ = nh_.advertise<nav_msgs::Odometry>("origin_odometry",1000);
    pub_new_odom_ = nh_.advertise<nav_msgs::Odometry>("new_odometry",1000);
    pub_rgb_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("color_cloud",1000);

}

Visualizer::~Visualizer(){}

void Visualizer::pubProjection(const Eigen::Matrix4d &transformation, const pcl::PointCloud<pcl::PointXYZ> &cloud, const cv::Mat &frame,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud,double t)
{
    ROS_INFO_STREAM("depth projection");
    cv::Mat HSV_MAT, tmp_HSV;

    cv::Mat img_color = frame.clone();
    cv::Mat depth_map = cv::Mat::zeros(frame.size(),CV_8UC1);

    std::vector<cv::Mat> planes;

    // //printf("convert 1 to 3...\n");
    // for(int i =0; i < 3; i++)
    // {
    //     planes.push_back(frame);
    // }
    // //printf("merge...\n");
    // cv::merge(planes, img_color);
    
    //printf("convert to hsv ...\n");
    cv::cvtColor(img_color, HSV_MAT, cv::COLOR_BGR2HSV);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(cloud, *cloud_out, transformation);

    //printf("projecting...\n");
    for(pcl::PointCloud<pcl::PointXYZ>::iterator pt = cloud_out->points.begin(); pt != cloud_out->points.end(); pt++)
    {
        if(pt->z < 0)
        {
            continue;
        }

        cv::Point2f xy = Point3DTo2D(*pt);

        if(xy.x> 0 && xy.x < frame.cols && xy.y > 0 && xy.y < frame.rows)
        {
            double depth = pt->z;//sqrt(pt->x * pt->x + pt->y * pt->y + pt->z * pt->z);
            
            //depth_map.at<uchar>(xy) = 100 - depth;
            depth_map.at<uchar>(xy.y, xy.x) = 100 - depth;
        
            double new_depth = clip(depth, 0,100);
    
            //printf("draw point...\n");
            cv::circle(HSV_MAT, xy, 3, cv::Scalar(int(new_depth * 6),255, 255), -1);
        }
    }

    cv::Mat SHOW_MAT, DEPTH_SHOW;
    cv::cvtColor(HSV_MAT, SHOW_MAT, cv::COLOR_HSV2BGR);

    depth_map_util.depthCompletionFast(depth_map);

    cv::Mat heat_map;
    cv::applyColorMap(depth_map, heat_map,cv::COLORMAP_JET);

    std_msgs::Header header;
    header.frame_id = "camera";
    header.stamp = ros::Time(t);

    cv_bridge::CvImage img_projection_msg;
    img_projection_msg.header = header;
    img_projection_msg.encoding = sensor_msgs::image_encodings::BGR8;
    img_projection_msg.image = SHOW_MAT.clone();
    pub_projection_.publish(img_projection_msg);

    cv_bridge::CvImage img_depth_msg;
    img_depth_msg.header = header;
    img_depth_msg.encoding = sensor_msgs::image_encodings::BGR8;
    img_depth_msg.image = heat_map.clone();
    pub_depth_map_.publish(img_depth_msg);

    for(int j =0; j < depth_map.rows; j++)
    {
        for(int i = 0; i < depth_map.cols; i++)
        {
            int depth_value = 100 - depth_map.at<uchar>(j,i);

            if(depth_value < 0.1)
            {
                continue;
            }

            if(depth_value > 99)
            {
                continue;
            }
            
            //std::cout << "depth value: " << depth_value << std::endl;

            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            cam_->liftProjective(a,b);

            pcl::PointXYZRGB point;
            point.x = depth_value * b.x()/b.z();
            point.y = depth_value * b.y()/b.z();
            point.z = depth_value;
            point.r = frame.at<cv::Vec3b>(j,i)[2];
            point.g = frame.at<cv::Vec3b>(j,i)[1];
            point.b = frame.at<cv::Vec3b>(j,i)[0];
            rgb_cloud->push_back(point);
        }
    }

    pubNewCloud(rgb_cloud,t);

    //cv::imshow("fill map", heat_map);
    //printf("showing...\n");
    //cv::imshow("projection mat", SHOW_MAT);
    //cv::waitKey(2);
}

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

    pub_new_odom_.publish(odometry);
}

void Visualizer::pubNewCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud, double t)
{
    sensor_msgs::PointCloud2 point_msg;

    pcl::toROSMsg(*rgb_cloud,point_msg);

    point_msg.header.frame_id = "camera";
    point_msg.header.stamp = ros::Time(t);
    
    pub_rgb_cloud_.publish(point_msg);
}

CloudVisualizer::CloudVisualizer():init(false){}

void CloudVisualizer::updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
     boost::mutex::scoped_lock lk(m);
    if(cloud->size() == 0)
    {
        ROS_INFO_STREAM(">>>>>>>no points<<<<<<<<");
        return;
    }

    if(!init)
    {
        viewer->addPointCloud(cloud, "cloud");
        init = true;

    }else
    {
        //viewer->removePointCloud("cloud");
        //viewer->addPointCloud(cloud, "cloud");
        viewer->updatePointCloud(cloud,"cloud");
    }
    
    
}

void CloudVisualizer::Spin()
{
    {
        boost::mutex::scoped_lock lk(m);
        viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("3D Debug Viewer");
        int v_pcl(0);
        viewer->createViewPort(0.,0.,1.,1.,v_pcl);
        viewer->setBackgroundColor(0, 0, 0,v_pcl);
        viewer->addCoordinateSystem(1.0,v_pcl);
        viewer->addText("3D RGB MAP", 10, 10, "debugger text", v_pcl);
        viewer->setCameraPosition(-3,0,0,0,0,0,v_pcl);
    }

    while(!viewer->wasStopped())
    {
        boost::mutex::scoped_lock lk(m);
        viewer->spinOnce();
        boost::this_thread::sleep(boost::posix_time::microseconds(10000));
    }
    
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

