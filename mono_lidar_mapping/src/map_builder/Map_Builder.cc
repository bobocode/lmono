#include "map_builder/Map_Builder.h"

double clip(double n, double lower, double upper)
{
    return std::max(lower, std::min(n, upper));
}

void MapBuilder::processMapping()
{
    while(true)
    {
        //printf("waiting rgb point\n");
        if(!rgb_points_buf.empty())
        {   
            map_mutex_.lock();
            pcl::PointCloud<pcl::PointXYZRGB> rgb_cloud = rgb_points_buf.front().second;
            double t = rgb_points_buf.front().first;
            rgb_points_buf.pop();

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
            // pcl::PointCloud<pcl::PointXYZRGB>::Ptr transfered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
            *rgb_cloud_ptr = rgb_cloud;

            /*// Create the filtering object
            // pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
            // sor.setInputCloud(rgb_cloud_ptr);
            // sor.setMeanK (100);
            // sor.setStddevMulThresh (1.0);
            // sor.filter (*filtered_cloud_ptr);

            if(last_rgb_cloud != nullptr)
            {
                pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
                // Set the input source and target
                icp.setInputSource(filtered_cloud_ptr);
                icp.setInputTarget(last_rgb_cloud);

                // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
                icp.setMaxCorrespondenceDistance (0.5);
                // Set the maximum number of iterations (criterion 1)
                icp.setMaximumIterations (50);
                // Set the transformation epsilon (criterion 2)
                icp.setTransformationEpsilon (1e-8);
                // Set the euclidean distance difference epsilon (criterion 3)
                icp.setEuclideanFitnessEpsilon (1);
                icp.align(*filtered_cloud_ptr);

                Eigen::Matrix4d target_to_source;
                target_to_source = icp.getFinalTransformation ().cast<double>();

                Eigen::Matrix4d source_to_target = target_to_source.inverse();

                pcl::transformPointCloud(*filtered_cloud_ptr, *transfered_cloud_ptr,source_to_target);

            }else
            {
                last_rgb_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
                *last_rgb_cloud = *filtered_cloud_ptr;
            }*/
            
            if(rgb_map != nullptr && map_index % 1 == 0)
            {
                *rgb_map += rgb_cloud;

            }else{
                rgb_map.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
                *rgb_map = rgb_cloud;
            }

            printf("mapping at %d\n",map_index);

            map_index++;

            //cloud_vis.updateCloud(rgb_map);

            sensor_msgs::PointCloud2 rgb_map_msg;
            pcl::toROSMsg(*rgb_map,rgb_map_msg);
            rgb_map_msg.header.frame_id = "camera_init";
            rgb_map_msg.header.stamp = ros::Time(t);
            pub_rgb_map_.publish(rgb_map_msg);

            if(map_index >0 && map_index % 10 ==0)
            {
                // pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
                // pcl::VoxelGrid<pcl::PointXYZRGB> vg;
                // vg.setInputCloud (rgb_map);
                // vg.setLeafSize (0.1f, 0.1f, 0.1f);//change leaf size into 0.5cm
                // vg.filter (*filtered_cloud_ptr);

                if(SAVE_MAP)
                {
                    std::string map_file_path = "/home/bo/raw_data/map/rgb_map" + std::to_string(map_index) + ".ply";
                    pcl::io::savePLYFileBinary(map_file_path, *rgb_map);
                }
                
                rgb_map->clear();
                rgb_map==nullptr;
            }

            map_mutex_.unlock();

        }

        std::chrono::milliseconds dura(10);
        std::this_thread::sleep_for(dura);
    }
    
}

/*void MapBuilder::alignmentToMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud_ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud_ptr,
                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr transfor_cloud_ptr)
{
    pcl::PointCloud<pcl::PointXYZRGB>& source_cloud = *source_cloud_ptr;
    pcl::PointCloud<pcl::PointXYZRGB>& target_cloud = *target_cloud_ptr;

    std::vector<int> nan_idx;
    pcl::removeNaNFromPointCloud(source_cloud, source_cloud, nan_idx);
    pcl::removeNaNFromPointCloud(target_cloud, target_cloud, nan_idx);
    
    // Estimate cloud normals
    std::cout << "Computing source cloud normals\n";
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> ne;
    pcl::PointCloud<pcl::PointNormal>::Ptr src_normals_ptr (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>& src_normals = *src_normals_ptr;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_xyz (new pcl::search::KdTree<pcl::PointXYZRGB>());
    ne.setInputCloud(source_cloud_ptr);
    ne.setSearchMethod(tree_xyz);
    ne.setRadiusSearch(0.05);
    ne.compute(*src_normals_ptr);
    for(size_t i = 0;  i < src_normals.points.size(); ++i) {
        src_normals.points[i].x = source_cloud.points[i].x;
        src_normals.points[i].y = source_cloud.points[i].y;
        src_normals.points[i].z = source_cloud.points[i].z;
    }

    std::cout << "Computing target cloud normals\n";
    pcl::PointCloud<pcl::PointNormal>::Ptr tar_normals_ptr (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>& tar_normals = *tar_normals_ptr;
    ne.setInputCloud(target_cloud_ptr);
    ne.compute(*tar_normals_ptr);
    for(size_t i = 0;  i < tar_normals.points.size(); ++i) {
        tar_normals.points[i].x = target_cloud.points[i].x;
        tar_normals.points[i].y = target_cloud.points[i].y;
        tar_normals.points[i].z = target_cloud.points[i].z;
    }

    const float min_scale = 0.1f;
    const int n_octaves = 6;
    const int n_scales_per_octave = 10;
    const float min_contrast = 0.5f;

    // Estimate the SIFT keypoints
    pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale>::Ptr src_keypoints_ptr (new pcl::PointCloud<pcl::PointWithScale>);
    pcl::PointCloud<pcl::PointWithScale>& src_keypoints = *src_keypoints_ptr;
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree_normal(new pcl::search::KdTree<pcl::PointNormal> ());
    sift.setSearchMethod(tree_normal);
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.setInputCloud(src_normals_ptr);
    sift.compute(src_keypoints);

    std::cout << "Found " << src_keypoints.points.size () << " SIFT keypoints in source cloud\n";

    pcl::PointCloud<pcl::PointWithScale>::Ptr tar_keypoints_ptr (new pcl::PointCloud<pcl::PointWithScale>);
    pcl::PointCloud<pcl::PointWithScale>& tar_keypoints = *tar_keypoints_ptr;
    sift.setInputCloud(tar_normals_ptr);
    sift.compute(tar_keypoints);

    cout << "Found " << tar_keypoints.points.size () << " SIFT keypoints in target cloud\n";

    // Extract FPFH features from SIFT keypoints
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_keypoints_xyz (new pcl::PointCloud<pcl::PointXYZRGB>);                           
    pcl::copyPointCloud (src_keypoints, *src_keypoints_xyz);
    pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
    fpfh.setSearchSurface (source_cloud_ptr);
    fpfh.setInputCloud (src_keypoints_xyz);
    fpfh.setInputNormals (src_normals_ptr);
    fpfh.setSearchMethod (tree_xyz);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr src_features_ptr (new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::PointCloud<pcl::FPFHSignature33>& src_features = *src_features_ptr;
    fpfh.setRadiusSearch(0.05);
    fpfh.compute(src_features);
    std::cout << "Computed " << src_features.size() << " FPFH features for source cloud\n";

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tar_keypoints_xyz (new pcl::PointCloud<pcl::PointXYZRGB>);                           
    pcl::copyPointCloud (tar_keypoints, *tar_keypoints_xyz);
    fpfh.setSearchSurface (target_cloud_ptr);
    fpfh.setInputCloud (tar_keypoints_xyz);
    fpfh.setInputNormals (tar_normals_ptr);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr tar_features_ptr (new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::PointCloud<pcl::FPFHSignature33>& tar_features = *tar_features_ptr;
    fpfh.compute(tar_features);
    cout << "Computed " << tar_features.size() << " FPFH features for target cloud\n";

    const float min_sample_dist = 0.5f;
    const float max_correspondence_dist = 1.0f;
    const int nr_iters = 30;

    // Compute the transformation matrix for alignment
    Eigen::Matrix4f tform = Eigen::Matrix4f::Identity();
    tform = computeInitialAlignment (src_keypoints_ptr, src_features_ptr, tar_keypoints_ptr,
          tar_features_ptr, min_sample_dist, max_correspondence_dist, nr_iters);

    //tform = refineAlignment(source_cloud_ptr, target_cloud_ptr, tform, );

    pcl::transformPointCloud(source_cloud, *transfor_cloud_ptr, tform);
    std::cout << "Calculated transformation\n";

}*/


void MapBuilder::associateToMap(const Eigen::Quaterniond &Q,const Eigen::Vector3d &T, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, 
                        cv::Mat &frame,const double t)
{
    cv::Mat HSV_MAT, tmp_HSV;

    cv::Mat img_color = frame.clone();
    cv::Mat depth_map = cv::Mat::zeros(frame.size(),CV_8UC1);

    cv::cvtColor(img_color, HSV_MAT, cv::COLOR_BGR2HSV);

    //printf("projecting...\n");
    for(pcl::PointCloud<pcl::PointXYZ>::iterator pt = cloud->points.begin(); pt != cloud->points.end(); pt++)
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

    //cv::imwrite("/home/bo/pro_img.png", SHOW_MAT);

    depthFill(depth_map);

    cv::Mat heat_map;
    cv::applyColorMap(depth_map, heat_map,cv::COLORMAP_JET);

    std_msgs::Header header;
    header.frame_id = "camera";
    header.stamp = ros::Time(t);

    cv_bridge::CvImage img_projection_msg;
    img_projection_msg.header = header;
    img_projection_msg.encoding = sensor_msgs::image_encodings::BGR8;
    img_projection_msg.image = SHOW_MAT.clone();
    pub_pro_img_.publish(img_projection_msg);

    cv_bridge::CvImage img_depth_msg;
    img_depth_msg.header = header;
    img_depth_msg.encoding = sensor_msgs::image_encodings::BGR8;
    img_depth_msg.image = heat_map.clone();
    pub_depth_map_.publish(img_depth_msg);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    for(int j =0; j < depth_map.rows; j++)
    {
        for(int i = 0; i < depth_map.cols; i++)
        {
            int depth_value = 100 - depth_map.at<uchar>(j,i);

            if(depth_value <= 0)
            {
                continue;
            }

            if(depth_value >= 70)
            {
                continue;
            }
            
            //std::cout << "depth value: " << depth_value << std::endl;

            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera->liftProjective(a,b);

            pcl::PointXYZRGB point;
            point.x = depth_value * b.x()/b.z();
            point.y = depth_value * b.y()/b.z();
            point.z = depth_value;
            point.r = frame.at<cv::Vec3b>(j,i)[2];
            point.g = frame.at<cv::Vec3b>(j,i)[1];
            point.b = frame.at<cv::Vec3b>(j,i)[0];

            if(abs(point.x) > 20 && point.y > 1.8)
            {
                continue;
            }

            rgb_cloud->push_back(point);
        }
    }

    //transfer to world frame C0
    Eigen::Matrix4d transfer_to_w;
    transfer_to_w.setIdentity();

    transfer_to_w.block<3,3>(0,0) = Q.toRotationMatrix();
    transfer_to_w.block<3,1>(0,3) = T;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr w_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*rgb_cloud, *w_cloud, transfer_to_w);

    rgb_points_buf.push(std::make_pair(t, *w_cloud));

    //processMapping();

    sensor_msgs::PointCloud2 point_msg;
    pcl::toROSMsg(*rgb_cloud,point_msg);
    point_msg.header.frame_id = "camera";
    point_msg.header.stamp = ros::Time(t);
    pub_rgb_points_.publish(point_msg);

}

void MapBuilder::depthFill(cv::Mat &DMap)
{
    cv::Mat kernel_mat;
    cv::Mat tmp_mat = DMap.clone();
    //int num = (kernel_size - 1 )/2 ;

    if(KERNEL_TYPE == "FULL")
    {
        kernel_mat = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(KERNEL_SIZE,KERNEL_SIZE));

    }else if(KERNEL_TYPE == "CROSS")
    {
        kernel_mat = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(KERNEL_SIZE,KERNEL_SIZE));

    }else
    {
        kernel_mat = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(KERNEL_SIZE,KERNEL_SIZE));
    }

    
    //dilate
    cv::Mat dilate_mat;
    cv::dilate(tmp_mat, dilate_mat,kernel_mat);

    //hole closing
    cv::Mat hole_fill;
    cv::morphologyEx(dilate_mat,hole_fill,cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT,cv::Size(KERNEL_SIZE,KERNEL_SIZE)));
    cv::dilate(hole_fill,dilate_mat,cv::getStructuringElement(cv::MORPH_RECT,cv::Size(7,7)));

    for(int k =0; k< hole_fill.rows; k++)
    {
        for(int q =0; q< hole_fill.cols; q++)
        {
            if(hole_fill.at<uchar>(k,q) < 0.1)
            {
                hole_fill.at<uchar>(k,q) = dilate_mat.at<uchar>(k,q);
            }
        }
    }

    //large fill
    /*cv::dilate(hole_fill,dilate_mat,cv::Mat::ones(31,31,CV_8UC1));
    for(int k =0; k< hole_fill.rows; k++)
    {
        for(int q =0; q< hole_fill.cols; q++)
        {
            if(hole_fill.at<uchar>(k,q) < 0.1)
            {
                hole_fill.at<uchar>(k,q) = dilate_mat.at<uchar>(k,q);
            }
        }
    }*/

    //extend highest to the top of image

    cv::medianBlur(hole_fill, tmp_mat,5);
    
    cv::Mat result;
    if(BLUR_TYPE == "bilateral")
    {
        cv::bilateralFilter(tmp_mat,result,5,1.5,2.0);
    }else
    {
        cv::GaussianBlur(tmp_mat,result, cv::Size(5,5), 0);
    }

    DMap = result.clone();
}

cv::Point2f MapBuilder::Point3DTo2D(const pcl::PointXYZ &pt)
{
    Eigen::Vector3d P;
    P.x() = pt.x;
    P.y() = pt.y;
    P.z() = pt.z;

    Eigen::Vector2d p_dst;
    m_camera->spaceToPlane(P,p_dst);

    return cv::Point2f(p_dst.x(), p_dst.y());
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
        viewer->setCameraPosition(0,0,7,0,0,0,v_pcl);
    }

    while(!viewer->wasStopped())
    {
        boost::mutex::scoped_lock lk(m);
        viewer->spinOnce();
        boost::this_thread::sleep(boost::posix_time::microseconds(10000));
    }
    
}
