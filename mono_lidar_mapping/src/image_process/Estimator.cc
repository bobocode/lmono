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
#include "image_process/Estimator.h"

Estimator::Estimator():feature_tracker{}, feature_manager{}
{
    ROS_INFO("initial beigins");
}

Estimator::~Estimator()
{
    printf("join thread \n");
}

void Estimator::setParameter()
{
    for(int i =0; i < CAM_NAMES.size(); i++)
    {
        std::string file_name = CAM_NAMES[i];
        readIntrinsicYml(file_name);
        feature_tracker.cams.push_back(camodocal::CameraFactory::instance()->generateCameraFromYamlFile(file_name));

        std::cout << "cam[" << i << "] calib information: " << std::endl;
        Eigen::Matrix<double, 3,4> projection;
        projection << cameras[i].fx, 0, cameras[i].cx, cameras[i].Tx,
            0, cameras[i].fy, cameras[i].cy, cameras[i].Ty,
            0,0,1,0;

        std::cout << projection << std::endl; 

    }

    if(OPEN_VISO)
    {
        get_lost = false;
        change_reference_frame = false;

        mono_visual_odometer_params_.calib.cu = cameras[0].cx;
        mono_visual_odometer_params_.calib.cv = cameras[0].cy;
        mono_visual_odometer_params_.calib.f = cameras[0].fx;
        mono_visual_odometer_params_.height = CAMERA_HEIGHT;
        mono_visual_odometer_params_.pitch = CAMERA_PITCH;

        mono_visual_odometer_.reset(new viso::VisualOdometryMono(mono_visual_odometer_params_));
    }

    visualizer_.reset(new Visualizer(feature_tracker.cams[0]));

    axxbsolver.InitialAXXBSolver();

    prev_time = -1;
    cur_time = 0;

    inputImageCnt = 0;
    frame_count = 0;

    for(int i =0; i < WINDOW_SIZE +1; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
    }

    for(int i =0; i < CAM_NAMES.size(); i++)
    {
        TLC[i].setIdentity();
    }

    L0_Pos.setIdentity();
    C0_Pos.setIdentity();
    prev_laser_pose.setIdentity();
    prev_cam_pose.setIdentity();

    stage_flag = NOT_INITED;

    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();

    feature_manager.clearState();

    Eigen::Matrix3d rot;
    rot <<   0.0519712,  0.0758698,   0.995762,
        -0.997109,  0.0592834,  0.0475246,
        -0.0554265,  -0.995354,  0.0787315;

    // // Eigen::Matrix3d rot;
    // // rot <<  -0.0648221, -0.00140819,  0.997896,
    // //         -0.997858,  0.00895319,  -0.064807,
    // //         -0.00884309,  -0.999959, -0.00198554;
    
    TLC[0].block<3,3>(0,0) = rot;
    ESTIMATE_LASER = 1;

    MonoProjectionFactor::sqrt_info = FACTOR_WEIGHT * Eigen::Matrix2d::Identity();
    LASERFactor::sqrt_info = LASER_W * FACTOR_WEIGHT * Eigen::Matrix<double,6,6>::Identity();
}

void Estimator::setupRos(ros::NodeHandle &nh)
{
    MeasurementManager::setupRos(nh);
    PointMapping::SetupRos(nh, false);
}

double Estimator::reprojectionError(Eigen::Matrix3d &Ri, Eigen::Vector3d &Pi, Eigen::Matrix3d &Rj,Eigen::Vector3d &Pj, Eigen::Vector2d &uvi, Eigen::Vector2d &uvj, double depth, Eigen::Matrix4d &EX_PARA)
{
    Eigen::Vector3d pt_i, pt_j;

    pt_i.head<2>() = uvi;
    pt_i.z() = 1.0;

    pt_j.head<2>() = uvj;
    pt_j.z() = 1.0;

    Eigen::Matrix3d Rlc0 = EX_PARA.block<3,3>(0,0);
    Eigen::Vector3d tlc0 = EX_PARA.block<3,1>(0,3);

    Eigen::Vector3d pt_c0_i = depth * pt_i;
    Eigen::Vector3d pt_l_i = Rlc0 * pt_c0_i + tlc0;

    Eigen::Vector3d pt_w = Ri * pt_l_i + Pi;
    Eigen::Vector3d pt_l_j = Rj.transpose() * (pt_w - Pj);

    Eigen::Vector3d pt_c0_j = Rlc0.transpose() *(pt_l_j - tlc0);

    //std::cout << "pt ci j: " << (pt_c1_j / pt_c1_j.z()).transpose() << std::endl;
    //std::cout << "pt j: " << pt_j.transpose() << std::endl;

    Eigen::Vector2d residual = (pt_c0_j / pt_c0_j.z()).head<2>() - pt_j.head<2>();
    
    return sqrt(residual.x() * residual.x() + residual.y() * residual.y());

}

void Estimator::outliersRejection(std::set<int> &removeIndex, const double &error)
{
    int feature_index = -1;
    int rejectCnt=0;
    double sum_err = 0;
    for(auto &it_per_id: feature_manager.feature)
    {
        double err = 0;
        int errCnt = 0;

        it_per_id.used_num = it_per_id.feature_per_frame.size();

        if(it_per_id.used_num < 4)
        {
            continue;
        }

        feature_index++;

        int i = it_per_id.start_frame, j = i -1;

        Eigen::Vector2d pt_i = it_per_id.feature_per_frame[0].pt;
        double depth = it_per_id.estimated_depth;
        //printf("outlier depth: %f\n", depth);
        //printf("starting frame %d: \n", i, it_per_id.used_num);
        for(auto &it_per_frame: it_per_id.feature_per_frame)
        {
            j++;
            if(i != j)
            {   
                //printf("mono two frames: \n");
                Eigen::Vector2d pt_j = it_per_frame.pt;
                double temp_err = reprojectionError(Rs[i], Ps[i],Rs[j], Ps[j], pt_i, pt_j, depth, TLC[0]);
                //printf("tracking frame %d: %f\n",j, temp_err);

                err += temp_err;
                errCnt++;
            }

            // if(it_per_frame.stereo)
            // {
            //     Eigen::Vector2d pt_j_right = it_per_frame.right_pt;

            //     if(i !=j)
            //     {   
            //        //printf("stereo two frames: \n");
            //         double tmp_error = reprojectionError(Rs[i], Ps[i],Rs[j], Ps[j], pt_i, pt_j_right, depth,rlc[0],CAM0_T_CAM1);
            //         err += tmp_error;
            //         errCnt++;
            //     }else
            //     {
            //         //printf("stereo one frame: \n");
            //         double tmp_error = reprojectionError(Rs[i], Ps[i],Rs[j], Ps[j],pt_i, pt_j_right,depth, rlc[0],CAM0_T_CAM1);
            //         err += tmp_error;
            //         errCnt++;
            //     }
                
            // }
            
        }

        double ave_err = err / errCnt;
        sum_err += ave_err;

        // fprintf(projection_error, "%d,%d,%d, %d, %f \n", inputImageCnt, it_per_id.feature_id, it_per_id.start_frame,it_per_id.used_num, ave_err * FACTOR_WEIGHT);
        // fflush(projection_error);

        //printf("reprojection error: %f\n", ave_err * FACTOR_WEIGHT);
        if(ave_err * FACTOR_WEIGHT > error)
        {   rejectCnt++;
            removeIndex.insert(it_per_id.feature_id);
        }

        //printf("feature %d: starting frame %d,used num %d\n", feature_index, it_per_id.start_frame, it_per_id.used_num);
        
    }
    int count = feature_manager.getFeatureCount();
    //printf("average reprojection error: %f\n", sum_err/count);
    //printf("outlier Num: %d\n", rejectCnt);
}

Eigen::Matrix4d Estimator::processCompactData(const sensor_msgs::PointCloud2ConstPtr &compact_data,
                                   const std_msgs::Header &header) 
{
    //1. process laser data
    PointMapping::CompactDataHandler(compact_data);

    TransformAssociateToMap();
    ROS_INFO_STREAM("transform original tobe \n" << transform_tobe_mapped_);

    /// 2. process decoded data
    PointMapping::Process();
    
    lclio::Transform transform_to_init_ = transform_aft_mapped_;

    Eigen::Matrix4d Pos;
    Pos.setIdentity();

    L0_Q = transform_to_init_.rot.cast<double>();
    L0_P = transform_to_init_.pos.cast<double>();

    Pos.block<3,3>(0,0) = L0_Q.toRotationMatrix();
    Pos.block<3,1>(0,3) = L0_P;
    ROS_INFO_STREAM("laser transformation to init: \n" << Pos.matrix());
    return Pos;

}

cv::Mat Estimator::getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    //state_mutex_.lock();
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        ROS_INFO_STREAM("mono8");
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);

    }else
        ROS_INFO_STREAM("image type " << img_msg->encoding);
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    
    printf("reading picture\n");

    cv::Mat img = ptr->image.clone();
    //state_mutex_.unlock();
    return img;
}

bool Estimator::visoHandler(const cv::Mat &img0, Eigen::Matrix4d &pos, const cv::Mat &img1)
{
    int32_t width = img0.cols;
    int32_t height = img0.rows;

    uint8_t* l_image_data;
    uint8_t* r_image_data;
    int l_step, r_step;

    l_image_data = img0.data;
    l_step = img0.step[0];
    int32_t dims[] = {width, height, width};

    pos.setIdentity();

    bool success;

    if(frame_count ==0)
    {
        success = mono_visual_odometer_->process(l_image_data, dims);
    }else
    {
        success = mono_visual_odometer_->process(l_image_data,dims,change_reference_frame);
    }

    if(success)
    {
        viso::Matrix delta_motion = viso::Matrix::inv(mono_visual_odometer_->getMotion());

        Eigen::Matrix3d rr;
        Eigen::Vector3d tt;

        rr << delta_motion.val[0][0],delta_motion.val[0][1], delta_motion.val[0][2],
                    delta_motion.val[1][0],delta_motion.val[1][1], delta_motion.val[1][2],
                    delta_motion.val[2][0], delta_motion.val[2][1],delta_motion.val[2][2];
                    
        tt << delta_motion.val[0][3], delta_motion.val[1][3], delta_motion.val[2][3];

        pos.block<3,3>(0,0) = rr;
        pos.block<3,1>(0,3) = tt;

    }else
    {
        if(frame_count !=0)
        {
            change_reference_frame = true;
        }
        
    }
    
    return success;
    
}

void Estimator::resetViso()
{
    frame_count = 0;
    prev_cam_pose.setIdentity();
    C0_Pos.setIdentity();
    all_image_frame.clear();
}

void Estimator::processImage(const double &header,const cv::Mat &img0, Eigen::Matrix4d &transform_to_init,const cv::Mat &img1)
{
    ROS_INFO_STREAM(">>>>>>>>>>>>>>>>>>>new image coming<<<<<<<<<<<<<<<<<<");
    std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 6, 1>>>> feature;

    image_l = img0.clone();

    tic_toc_.Tic();
    feature = feature_tracker.trackImage(header, image_l);
    ROS_INFO_STREAM("feature tracker time: " << tic_toc_.Toc());

    cv::Mat imgTrack = feature_tracker.getTrackImage();
    visualizer_->pubTrackImg(imgTrack, header);

    if(feature_manager.featureCheck(frame_count, feature, header))
    {
        marginalization_flag = MARGIN_OLD;
        printf("keyframe\n");

    }else
    {
        marginalization_flag = MARGIN_SECOND_NEW;
        printf("non-keyframe\n");
    }

    Eigen::Matrix4d pos;
    pos.setIdentity();

    if(OPEN_VISO)
    {
        if(visoHandler(image_l, pos))
        {
            C0_Pos = prev_cam_pose * pos;

        }else
        {
            ROS_WARN_STREAM("viso odometry lost");
            get_lost = true;
            resetViso();
            visoHandler(image_l,pos);
        }
    }

    ImageFrame imageframe(feature, header,transform_to_init, C0_Pos);
    all_image_frame.push_back(std::make_pair(header, imageframe));

    if(ESTIMATE_LASER == 2)
    {
        Eigen::Matrix4d laser_delta_pose = prev_laser_pose.inverse() * transform_to_init;
        Eigen::Quaterniond laser_delta_q = Eigen::Quaterniond(laser_delta_pose.block<3,3>(0,0));
        
        if(frame_count !=0 && !OPEN_VISO)
        {
            std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> corres = feature_manager.getCorresponding(frame_count-1, frame_count);
            Matrix3d calib_rcl;
            Eigen::Quaterniond camera_delta_q = Eigen::Quaterniond(axxbsolver.solveRelativeR(corres));

            ROS_INFO_STREAM("delta_cam_q: \n" << camera_delta_q.toRotationMatrix());
            ROS_INFO_STREAM("delta_laser_q: \n" << laser_delta_q.toRotationMatrix());

            if(axxbsolver.CalibrationExRotation(camera_delta_q,laser_delta_q, calib_rcl, 10))
            {
                ROS_WARN_STREAM("initial extrinsic rotation calib between cam0 and laser success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_rcl);

                TLC[0].block<3,3>(0,0) = calib_rcl;
                TLC[1].block<3,3>(0,0) = calib_rcl;

                ESTIMATE_LASER = 1;
            }

        }

        if(frame_count !=0 && OPEN_VISO)
        {
            Matrix3d calib_rcl;
            Eigen::Quaterniond camera_delta_q = Eigen::Quaterniond(pos.block<3,3>(0,0));

            ROS_INFO_STREAM("delta_cam_q: \n" << camera_delta_q.toRotationMatrix());
            ROS_INFO_STREAM("delta_laser_q: \n" << laser_delta_q.toRotationMatrix());


            if(axxbsolver.CalibrationExRotation(camera_delta_q,laser_delta_q, calib_rcl, 10))
            {
                ROS_WARN_STREAM("initial extrinsic rotation calib between cam0 and laser success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_rcl);

                TLC[0].block<3,3>(0,0) = calib_rcl.transpose();
                TLC[1].block<3,3>(0,0) = calib_rcl.transpose();

                ESTIMATE_LASER = 1;
            }

        }
    }
    
    if(stage_flag == NOT_INITED)
    {
        if(frame_count == WINDOW_SIZE)
        {
            if(ESTIMATE_LASER !=2)
            {
                if(runInitialization())
                {
                    //check();
                    optimization();
                    //check();
                    stage_flag = INITED;
                    std::set<int> removeIndex;
                    outliersRejection(removeIndex,3);
                    feature_manager.removeOutlier(removeIndex);
                    
                    ROS_INFO("Initialization finish!");
                    slideWindow();
                }else
                {
                    slideWindow();
                }
            }else
            {
                slideWindow();
            }
        }

        if(frame_count < WINDOW_SIZE)
        {
            frame_count++;
            int prev_frame  = frame_count -1;
            Ps[frame_count] = Ps[prev_frame];
            Rs[frame_count] = Rs[prev_frame];
        }

    }else
    {
        feature_manager.triangulate(frame_count, Rs, Ps, TLC[0]);
        std::set<int> removeIndex;
        //outliersRejection(removeIndex,100.0);
        ///feature_manager.removeOutlier(removeIndex);
        //check();
        optimization();
        //check();
        //ROS_WARN_STREAM("removing outliers");
        outliersRejection(removeIndex,3);
        feature_manager.removeOutlier(removeIndex);
        slideWindow();
        feature_manager.removeFailures();
    }

    prev_time = header;
    prev_cam_pose = C0_Pos;
}

void Estimator::processEstimation()
{
    while(1)
    {
        PairMeasurements measurements;
        std::unique_lock<std::mutex> buf_lk(buf_mutex_);
        con_.wait(buf_lk, [&] {
        return (measurements = GetMeasurements()).size() != 0;
        });
        buf_lk.unlock();
        ROS_INFO_STREAM("measurements obtained: " << measurements.size());

        thread_mutex_.lock();
        for(auto &measurement: measurements)
        {   
            ROS_INFO_STREAM("---------------Solving frame count--------" << frame_count);
            sensor_msgs::PointCloud2ConstPtr pts_msg = measurement.second;
            ROS_INFO_STREAM("processing laser data with stamp " << pts_msg->header.stamp.toSec());
            L0_Pos = this->processCompactData(pts_msg, pts_msg->header);

            sensor_msgs::ImageConstPtr img0_msg = measurement.first;
           
            image_l = getImageFromMsg(img0_msg);
            ROS_INFO_STREAM("processing image dara with stamp " << img0_msg->header.stamp.toSec());
            
            //ROS_INFO_STREAM("showing the picture");
            //cv::imshow("imshow", image_l);

            processImage(img0_msg->header.stamp.toSec(), image_l, L0_Pos, cv::Mat());

            if(ESTIMATE_LASER !=2 && stage_flag == INITED)
            {
                cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::fromROSMsg(*pts_msg, *cloud);
                //ROS_INFO_STREAM("transform pts to camera frame");
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                visualizer_->pubProjection(TLC[0].inverse(), *cloud, image_l,tmp_rgb_cloud,img0_msg->header.stamp.toSec());
                //cloud_vis.updateCloud(tmp_rgb_cloud);
            }
            prev_laser_pose = L0_Pos;
        }
        thread_mutex_.unlock();
    } 
}

bool Estimator::readIntrinsicYml(const std::string& filename)
{
    camera_calib camera;
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    //printf("reading camera Mat...");
    if(!fs.isOpened())
    {   
        printf("camera file does not exist.");
        return false;
    }

    if (!fs["model_type"].isNone())
    {
        fs["model_type"] >> camera.modelType;

        if (camera.modelType.compare("PINHOLE") != 0)
        {
            return false;
        }
    }

    fs["camera_name"] >> camera.camera_name;
    camera.image_width = static_cast<int>(fs["image_width"]);
    camera.image_height = static_cast<int>(fs["image_height"]);

    cv::FileNode n = fs["distortion_parameters"];
    camera.k1 = static_cast<double>(n["k1"]);
    camera.k2 = static_cast<double>(n["k2"]);
    camera.p1 = static_cast<double>(n["p1"]);
    camera.p2 = static_cast<double>(n["p2"]);

    n = fs["projection_parameters"];
    camera.fx = static_cast<double>(n["fx"]);
    camera.fy = static_cast<double>(n["fy"]);
    camera.cx = static_cast<double>(n["cx"]);
    camera.cy = static_cast<double>(n["cy"]);
    camera.Tx = static_cast<double>(n["tx"]);
    camera.Ty = static_cast<double>(n["ty"]);

    cameras.push_back(camera);
    
    return true;
}

void Estimator::slideWindow()
{
    if(marginalization_flag == MARGIN_OLD)
    {
        printf("margin the oldest frame\n");
        double t_0 = Header[0];
        back_R0 = Rs[0];
        back_P0 = Ps[0];

        if(frame_count == WINDOW_SIZE)
        {
            for(int i =0; i < frame_count; i++)
            {
                Header[i] = Header[i+1];
                Rs[i].swap(Rs[i+1]);
                Ps[i].swap(Ps[i+1]);
            }

            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE-1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE-1];
            Header[WINDOW_SIZE] = Header[WINDOW_SIZE-1];

            all_image_frame.erase(all_image_frame.begin());

            slideWindowOld();
        }
    }else
    {
        printf("margin the second new frame\n");
        if(frame_count == WINDOW_SIZE)
        {
            Header[frame_count-1] = Header[frame_count];
            Ps[frame_count-1] = Ps[frame_count];
            Rs[frame_count-1] = Rs[frame_count];

            all_image_frame.erase(all_image_frame.end()-1);
            
            slideWindowNew();
        }
    }

}

void Estimator::slideWindowOld()
{
    if(stage_flag == NOT_INITED)
    {
        feature_manager.removeBack();
    }else
    {
        Eigen::Matrix3d R0, R1;
        Eigen::Vector3d P0, P1;

        R0 = back_R0 * TLC[0].block<3,3>(0,0);
        R1 = Rs[0] * TLC[0].block<3,3>(0,0);

        P0 = back_P0 + back_R0 * TLC[0].block<3,1>(0,3);
        P1 = Ps[0] + Rs[0] * TLC[0].block<3,1>(0,3);

        feature_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }

}

void Estimator::slideWindowNew()
{
    feature_manager.removeFront(frame_count);
}

bool Estimator::solveInitialEx(std::vector<std::pair<double, ImageFrame>> all_image_frame,Eigen::VectorXd &x)
{
    Eigen::MatrixXd A{4,4};
    Eigen::VectorXd b{4};

    Eigen::Vector3d delta_bg;
    Eigen::Vector3d delta_bt;

    A.setZero(); 
    b.setZero();

    Eigen::Matrix3d rlc = TLC[0].block<3,3>(0,0);
    Eigen::Vector3d tlc;

    for(int i =0; i < frame_count; i++)
    {
        Eigen::MatrixXd tmp_A(4,4);
        tmp_A.setZero();
        Eigen::VectorXd tmp_b(4);
        tmp_b.setZero();

        Eigen::Matrix3d cam_Ri = all_image_frame[i].second.R;
        Eigen::Vector3d cam_Pi = all_image_frame[i].second.T;

        Eigen::Matrix3d laser_Ri = all_image_frame[i].second.L0_R;
        Eigen::Vector3d laser_Pi = all_image_frame[i].second.L0_T;

        int j = i+1;
        Eigen::Matrix3d cam_Rj = all_image_frame[j].second.R;
        Eigen::Vector3d cam_Pj = all_image_frame[j].second.T;

        Eigen::Matrix3d laser_Rj = all_image_frame[j].second.L0_R;
        Eigen::Vector3d laser_Pj = all_image_frame[j].second.L0_T;

        Eigen::Matrix3d cam_Rij = cam_Ri.transpose() * cam_Rj;
        Eigen::Matrix3d laser_Rij = laser_Ri.transpose() * laser_Rj;

        Eigen::Vector3d cam_Pij = cam_Ri.transpose() * (cam_Pj - cam_Pi);
        Eigen::Vector3d laser_Pij = laser_Ri.transpose() * (laser_Pj - laser_Pi);

        //ROS_INFO_STREAM("sfm R" << i <<" to R" << j <<": \n" << cam_Rij);
        //ROS_INFO_STREAM("sfm T" << i <<" to T" << j << ": \n" << cam_Pij.transpose());

        //ROS_INFO_STREAM("laser R" << i <<" to R" << j << ": \n"<< laser_Rij);
        //ROS_INFO_STREAM("laser T" << i <<" to T" << j << ": \n" << laser_Pij.transpose());

        // tmp_A.block<3,3>(0,0) = Eigen::Matrix3d::Zero();//-(laser_Rij.normalized() - Eigen::Matrix3d::Identity());// 
        // tmp_A.block<3,1>(0,3) = rlc * cam_Pij /100;

        // tmp_b.block<3,1>(0,0) = laser_Pij;

        tmp_A.block<3,3>(0,0) = (Eigen::Matrix3d::Identity() - cam_Rij.normalized()) * rlc.transpose();
        tmp_A.block<3,1>(0,3) = cam_Pij / 100;
        tmp_b.block<3,1>(0,0) = rlc.transpose() * laser_Pij;

        A += tmp_A.transpose() * tmp_A;
        b += tmp_A.transpose() * tmp_b;
    }
    A = A * 1000.0;
    b = b * 1000.0;
    x = A.ldlt().solve(b);
    tlc = x.segment<3>(0);
    double s = x(3) / 100.0;
    x(3) = s;
    std::cout << "Iniitial tlc: " << tlc.transpose() << std::endl;
    std::cout << (-rlc.transpose()* tlc).transpose() << std::endl;
    std::cout << "scale: " << s << std::endl;

    if(s<=0)
    {
        return false;
    }else
    {
        return true;
    }
    

}

bool Estimator::runInitialization()
{
    if(!OPEN_VISO)
    {
        Eigen::Quaterniond Q[frame_count+1];
        Eigen::Vector3d T[frame_count+1];

        std::map<int, Eigen::Vector3d> sfm_tracked_points;
        std::vector<SFMFeature> sfm_f;

        for(auto &it_per_id: feature_manager.feature)
        {
            int j = it_per_id.start_frame -1;
            SFMFeature tmp_feature;
            tmp_feature.state = false;
            tmp_feature.id = it_per_id.feature_id;

            for(auto &it_per_frame: it_per_id.feature_per_frame)
            {
                j++;
                Eigen::Vector3d pt_j(it_per_frame.pt.x(), it_per_frame.pt.y(), 1.0);
                tmp_feature.observation.push_back(std::make_pair(j, it_per_frame.pt));
            }

            sfm_f.push_back(tmp_feature);
        }

        Eigen::Matrix3d relative_R;
        Eigen::Vector3d relative_T;
        int l;
        if(!feature_manager.relativePose(relative_R, relative_T, l))
        {
            ROS_WARN_STREAM("Not enough features or parallax.");
            return false;
        }

        GlobalSFM sfm;
        if(!sfm.construct(frame_count+1, Q, T, l , relative_R, relative_T, sfm_f, sfm_tracked_points))
        {
            ROS_WARN_STREAM("global SFM failed!");
            marginalization_flag = MARGIN_OLD;
            return false;
        }

        //solve pnp for all frame
        map<int, Vector3d>::iterator it;
        for(int i =0; i < frame_count+1; i++)
        {
            cv::Mat r, rvec, t, D, tmp_r;

            // if(all_image_frame[i].first == Header[i])
            // {
            //     all_image_frame[i].second.R = Q[i].toRotationMatrix();
            //     all_image_frame[i].second.T = T[i];

            //     continue;
            // }

            Eigen::Matrix3d R_initial = (Q[i].inverse()).toRotationMatrix();
            Eigen::Vector3d P_initial = -R_initial * T[i];

            cv::eigen2cv(R_initial, tmp_r);
            cv::Rodrigues(tmp_r, rvec);
            cv::eigen2cv(P_initial,t);

            std::vector<cv::Point3f> pts_3_vector;
            std::vector<cv::Point2f> pts_2_vector;

            for(auto &id_pts: all_image_frame[i].second.points)
            {
                int feature_id = id_pts.first;
                for(auto &i_p: id_pts.second)
                {
                    it = sfm_tracked_points.find(feature_id);
                    if(it != sfm_tracked_points.end())
                    {
                        Eigen::Vector3d world_pts = it->second;
                        cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                        pts_3_vector.push_back(pts_3);
                        Eigen::Vector2d img_pts = i_p.second.head<2>();
                        cv::Point2f pts_2(img_pts(0), img_pts(1));
                        pts_2_vector.push_back(pts_2);
                    }
                }
            }

             cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);     
            if(pts_3_vector.size() < 6)
            {
                cout << "pts_3_vector size " << pts_3_vector.size() << endl;
                ROS_DEBUG("Not enough points for solve pnp !");
                return false;
            }
            if (! cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
            {
                ROS_DEBUG("solve pnp fail!");
                return false;
            }
            cv::Rodrigues(rvec, r);
            MatrixXd R_pnp,tmp_R_pnp;
            cv::cv2eigen(r, tmp_R_pnp);
            R_pnp = tmp_R_pnp.transpose();
            MatrixXd T_pnp;
            cv::cv2eigen(t, T_pnp);
            T_pnp = R_pnp * (-T_pnp);
            all_image_frame[i].second.R = R_pnp ;
            all_image_frame[i].second.T = T_pnp;

        }
    }

    Eigen::VectorXd x;

    if(!solveInitialEx(all_image_frame,x))
    {
        ROS_WARN_STREAM("solve s failed!");
        return false;
    }

    double s = x(3);

    Eigen::Matrix3d rlc = TLC[0].block<3,3>(0,0);
    Eigen::Vector3d tlc = x.segment<3>(0);//TLC[0].block<3,1>(0,3);
    TLC[0].block<3,1>(0,3) = tlc;

    prior_trans = TLC[0];

    Eigen::Matrix3d cam_R0 = all_image_frame[0].second.R;
    Eigen::Vector3d cam_P0 = all_image_frame[0].second.T;
    Eigen::Vector3d laser_P0 = all_image_frame[0].second.L0_T;

    for(int i =0; i <=frame_count;i++)
    {
        Eigen::Matrix3d cam_Ri = all_image_frame[i].second.R;
        Eigen::Vector3d cam_Pi = all_image_frame[i].second.T;

        // Rs[i] = cam_Ri * rlc.transpose();
        // Ps[i] = s * cam_Pi - Rs[i] * tlc - (s * cam_P0 - cam_R0 * rlc.transpose() * tlc);

        all_image_frame[i].second.T = s * cam_Pi;

        Eigen::Matrix3d L0_Ri = all_image_frame[i].second.L0_R;
        Eigen::Vector3d L0_Pi = all_image_frame[i].second.L0_T;

        Rs[i] = rlc.transpose() * L0_Ri;
        Ps[i] = rlc.transpose() * (L0_Pi - tlc) - rlc.transpose() * (laser_P0 - tlc);

        //std::cout << "Rs[" << i << "]\n" << Rs[i] << std::endl;
        //std::cout << "Ps[" <<i <<"]" << Ps[i].transpose() << std::endl;

        //std::cout << "L0_R[" << i << "]\n" << L0_Ri << std::endl;
        //std::cout << "L0_P[" << i << "]" << (L0_Pi - laser_P0).transpose() << std::endl;
    }

    feature_manager.clearDepth();
    feature_manager.triangulate(frame_count,Rs,Ps,TLC[0]);

    std::set<int> removeIndex;
    outliersRejection(removeIndex, 100.0);
    feature_manager.removeOutlier(removeIndex);
    //outliersRejection(removeIndex, 3.0);

    //optimization();
    //check();

    return true;

}

void Estimator::matrix2Double()
{
    for(int i=0; i <=WINDOW_SIZE; i++)
    {
        Eigen::Vector3d t_i;
        Eigen::Quaterniond q_i;

        t_i = Ps[i];
        q_i = Eigen::Quaterniond{Rs[i]};
        
        para_pose[i][0] = t_i.x();
        para_pose[i][1] = t_i.y();
        para_pose[i][2] = t_i.z();

        para_pose[i][3] = q_i.x();
        para_pose[i][4] = q_i.y();
        para_pose[i][5] = q_i.z();
        para_pose[i][6] = q_i.w();
        
    }

    Eigen::Vector3d t_ex1 = TLC[0].block<3,1>(0,3);
    Eigen::Quaterniond q_ex1{TLC[0].block<3,3>(0,0)};
    para_ex[0][0] = t_ex1.x();
    para_ex[0][1] = t_ex1.y();
    para_ex[0][2] = t_ex1.z();

    para_ex[0][3] = q_ex1.x();
    para_ex[0][4] = q_ex1.y();
    para_ex[0][5] = q_ex1.z();
    para_ex[0][6] = q_ex1.w();

    Eigen::VectorXd dep = feature_manager.getDepthVector();

    for(int i =0; i < feature_manager.getFeatureCount(); i++)
    {
        //printf("feature %d depth: %f\n",i,1.0/dep(i));
        para_depth_inv[i][0] = dep(i);
    }
    //std::cout << "feature size: " << dep.size() << std::endl;

}

void Estimator::double2Matrix()
{
    Eigen::Vector3d origin_R0 = mathutils::R2ypr(Rs[0]);
    Eigen::Vector3d origin_t0 = Ps[0];

    Eigen::Vector3d origin_R00 = mathutils::R2ypr(Eigen::Quaterniond(para_pose[0][6], para_pose[0][3], para_pose[0][4], para_pose[0][5]).toRotationMatrix());
    double yaw_diff = origin_R0.x() - origin_R00.x();
    double pitch_diff = origin_R0.y() - origin_R00.y();
    double roll_diff = origin_R0.z() - origin_R00.z();

    Eigen::Matrix3d rot_diff = mathutils::ypr2R(Eigen::Vector3d(0, 0, 0));
    //Eigen::Matrix3d rot_diff = Rs[0].transpose() * Eigen::Quaterniond(para_pose[0][6], para_pose[0][3], para_pose[0][4], para_pose[0][5]).toRotationMatrix();

    //ROS_DEBUG("euler singular point!");
    rot_diff = Rs[0] * Eigen::Quaterniond(para_pose[0][6],
                                    para_pose[0][3],
                                    para_pose[0][4],
                                    para_pose[0][5]).toRotationMatrix().transpose();
    

    std::cout << "rot diff: " << (origin_R0 - origin_R00).transpose() << std::endl;

    for(int i=0; i <= WINDOW_SIZE; i++)
    {
        Eigen::Vector3d t_i(para_pose[i][0]-para_pose[0][0], para_pose[i][1]-para_pose[0][1], para_pose[i][2]-para_pose[0][2]);
        Eigen::Quaterniond q_i(para_pose[i][6], para_pose[i][3], para_pose[i][4], para_pose[i][5]);

        //laser_pose[i].block<3,1>(0,3) =  rot_diff * t_i + origin_t0;
        //laser_pose[i].block<3,3>(0,0) =  rot_diff * q_i.normalized().toRotationMatrix();

        //ROS_INFO_STREAM("Ps[" << i << "] before: " << (Ps[i]).transpose());
        Ps[i] =  rot_diff * t_i + origin_t0;
        Rs[i] =  rot_diff * q_i.normalized().toRotationMatrix();
        //ROS_INFO_STREAM("Ps[" << i << "] after: " << Ps[i].transpose());
    }

    if(stage_flag == INITED)
    {
        lclio::Transform trans_prev(Eigen::Quaterniond(Rs[WINDOW_SIZE-1]).cast<float>(),
                                        Ps[WINDOW_SIZE-1].cast<float>());
        lclio::Transform trans_curr(Eigen::Quaterniond(Rs[WINDOW_SIZE]).cast<float>(),
                                        Ps[WINDOW_SIZE].cast<float>());

        lclio::Transform d_trans = trans_prev.inverse() * trans_curr;

        ROS_INFO_STREAM("incre in laser world pos" << d_trans.pos.transpose());
        ROS_INFO_STREAM("incre in laser world rot" << d_trans.rot.toRotationMatrix());

        // transform_tobe_mapped_bef_ = transform_tobe_mapped_ *  d_trans;
        // transform_tobe_mapped_ = transform_tobe_mapped_bef_;

       // TransformUpdate();
    }

    Eigen::Quaterniond q_10(Rs[10]);

    // fprintf(odometry, "%d,%f,%f,%f , %f, %f, %f, %f \n", inputImageCnt,Ps[10].x() , Ps[10].y(),Ps[10].z(),
    //                                                             q_10.w(), q_10.x(), q_10.y(), q_10.z());
    // fflush(odometry);
    {
        Eigen::Vector3d t_ex1(para_ex[0][0], para_ex[0][1], para_ex[0][2]);
        Eigen::Quaterniond q_ex1(para_ex[0][6], para_ex[0][3], para_ex[0][4], para_ex[0][5]);

        transform_lc_.pos = t_ex1.cast<float>();
        transform_lc_.rot = q_ex1.cast<float>();

        // fprintf(extrinsic, "%d,%f,%f,%f , %f, %f, %f, %f \n", inputImageCnt,t_ex1.x() ,t_ex1.y(),t_ex1.z(),
        //                                                             q_ex1.w(), q_ex1.x(), q_ex1.y(), q_ex1.z());
        // fflush(extrinsic);

        TLC[0].block<3,1>(0,3) = t_ex1;
        TLC[0].block<3,3>(0,0) = q_ex1.toRotationMatrix();

    }

    ROS_WARN_STREAM("extrinsic parameters: TLC" << endl << TLC[0]);
    //ROS_WARN_STREAM("extrinsic parameters: tlc " << t_ex1.transpose());

    Eigen::VectorXd dep = feature_manager.getDepthVector();
    for(int i =0; i < feature_manager.getFeatureCount(); i++)
    {   
        //ROS_INFO_STREAM("depth before: " << 1.0/ dep(i));
        dep(i) = para_depth_inv[i][0];
        //ROS_INFO_STREAM("depth after: " << 1.0/ dep(i));
    }

    feature_manager.setDepth(dep);
    feature_manager.removeFailures();
}

bool Estimator::optimization()
{
    matrix2Double();
    
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::CauchyLoss(1.0);
    
    ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
    problem.AddParameterBlock(para_ex[0], 7, local_parameterization);

    for(int i=0; i <=frame_count; ++i)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_pose[i], 7, local_parameterization);

        //Eigen::Matrix3d L0_Ri = all_image_frame[i].second.L0_R;
        //Eigen::Vector3d L0_Pi = all_image_frame[i].second.L0_T;
    }
    //printf("adding pose\n");
    if(last_marginalization_info && last_marginalization_info->valid)
    {
        //printf("add marginalization parameters...\n");
        Marginalization *marginalization = new Marginalization(last_marginalization_info);
        problem.AddResidualBlock(marginalization, NULL, last_marginalization_parameter_blocks);
    }
    //printf("adding margin\n");

    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();

        prior_trans = TLC[0];
        PriorFactor *f_prior = new PriorFactor(prior_trans);
        problem.AddResidualBlock(f_prior, NULL, para_ex[0]);
    }
    //printf("adding prior\n");

    for(int i =0; i < frame_count; ++i)
    {   
        Eigen::Matrix3d L0_Ri = all_image_frame[i].second.L0_R;
        Eigen::Vector3d L0_Pi = all_image_frame[i].second.L0_T;

        int j = i+1;

        Eigen::Matrix3d L0_Rj = all_image_frame[j].second.L0_R;
        Eigen::Vector3d L0_Pj = all_image_frame[j].second.L0_T;

        LASERFactor* laser_factor = new LASERFactor(L0_Ri, L0_Rj, L0_Pi, L0_Pj);
        problem.AddResidualBlock(laser_factor, NULL, para_pose[i], para_pose[j]);
    }
    //printf("adding laser factor\n");

    int feature_index = -1;
    for(auto &it_per_id: feature_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        
        if(it_per_id.used_num < 4)
        {
            continue;
        }

        ++feature_index;
        //printf("feature idx %d\n",feature_index);

        int i = it_per_id.start_frame;
        int j = i -1;

        problem.AddParameterBlock(para_depth_inv[feature_index],1);
        Eigen::Vector2d pt_i = it_per_id.feature_per_frame[0].pt;

        for(auto &it_per_frame : it_per_id.feature_per_frame)
        {
            j++;

            if(i != j)
            {   
                Eigen::Vector2d pt_j = it_per_frame.pt;
                //printf("frame j %d, pt j: [%f, %f]\n", j, pt_j.x(), pt_j.y());
                MonoProjectionFactor *f_mono = new MonoProjectionFactor(pt_i, pt_j,it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                problem.AddResidualBlock(f_mono, loss_function, para_ex[0],para_pose[i], para_pose[j],para_depth_inv[feature_index]);
            }

            // if(it_per_frame.stereo)
            // {
            //     Eigen::Vector2d pt_j_right = it_per_frame.right_pt;

            //     if(i !=j)
            //     {
            //        StereoTwoFramesResidualInfo *f = new StereoTwoFramesResidualInfo(pt_i, pt_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.right_velocity,
            //                                                             it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td,CAM0_T_CAM1);
            //        problem.AddResidualBlock(f, loss_function,para_ex[0], para_pose[i], para_pose[j], para_depth_inv[feature_index]);

            //     }else
            //     {
            //         StereoOneFrameResidualInfo *f = new StereoOneFrameResidualInfo(pt_i, pt_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.right_velocity,
            //                                                     it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td, CAM0_T_CAM1);
            //         problem.AddResidualBlock(f, loss_function, para_depth_inv[feature_index]);
            //     }
                
            // }
            
        }

    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.trust_region_strategy_type = ceres::DOGLEG;

    //options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = NUM_ITERATIONS;
    options.minimizer_progress_to_stdout = false;
    
    if(marginalization_flag = MARGIN_OLD)
    {
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    }else
    {
        options.max_solver_time_in_seconds = SOLVER_TIME;
    }

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    //std::cout << "summary report: " << summary.FullReport() << std::endl;

    double2Matrix();

    if(frame_count < WINDOW_SIZE)
    {
        return false;
    }

    margin();

    if (summary.termination_type == ceres::CONVERGENCE || summary.final_cost < 5e-03)
	{
		cout << "optimization converge" << endl;
	}
	else
	{
		cout << "optimization not converge " << endl;
        return false;
	}

    return true;

}

void Estimator::margin()
{
    ceres::LossFunction *loss_function;
    loss_function = new ceres::CauchyLoss(1.0);
    if(marginalization_flag == MARGIN_OLD)
    {   
        //printf("margin the oldest one\n");
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        matrix2Double();

        if(last_marginalization_info && last_marginalization_info->valid)
        {
            std::vector<int> drop_set;
            for(int i =0; i< static_cast<int>(last_marginalization_parameter_blocks.size());i++)
            {
                if(last_marginalization_parameter_blocks[i] == para_pose[0])
                {
                    drop_set.push_back(i);
                    //printf("adding margin pose\n");

                }
            }

            Marginalization *marginalization = new Marginalization(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization, NULL, last_marginalization_parameter_blocks, drop_set);
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        {
            Eigen::Matrix3d L0_Ri = all_image_frame[0].second.L0_R;
            Eigen::Vector3d L0_Pi = all_image_frame[0].second.L0_T;

            Eigen::Matrix3d L0_Rj = all_image_frame[1].second.L0_R;
            Eigen::Vector3d L0_Pj = all_image_frame[1].second.L0_T;

            LASERFactor* laser_factor = new LASERFactor(L0_Ri, L0_Rj, L0_Pi, L0_Pj);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(laser_factor, NULL, 
                                                            std::vector<double *>{para_pose[0], para_pose[1]},
                                                            std::vector<int>{0});
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        int feature_index = -1;
        for(auto &it_per_id: feature_manager.feature)
        {
            it_per_id.used_num = it_per_id.feature_per_frame.size();
            if(it_per_id.used_num < 4)
            {
                continue;
            }

            
            ++feature_index;
            int i = it_per_id.start_frame, j = i -1;

            if(i !=0)
            {
                continue;
            }

            //printf("add margin feature\n");
            Eigen::Vector2d pt_i = it_per_id.feature_per_frame[0].pt;

            for(auto &it_per_frame: it_per_id.feature_per_frame)
            {
                j++;
                if(i != j)
                {
                    Eigen::Vector2d pt_j = it_per_frame.right_pt;
                    MonoProjectionFactor *f = new MonoProjectionFactor(pt_i, pt_j,it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function, std::vector<double *>{para_ex[0],para_pose[i], para_pose[j], para_depth_inv[feature_index]},std::vector<int>{1,3});
                    marginalization_info->addResidualBlockInfo(residual_block_info);
                }

                // if(it_per_frame.stereo)
                // {
                //     Eigen::Vector2d pt_j_right = it_per_frame.right_pt;

                //     if(i != j)
                //     {
                //         StereoTwoFramesResidualInfo *f = new StereoTwoFramesResidualInfo(pt_i, pt_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.right_velocity,
                //                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td,CAM0_T_CAM1);
                //         ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function, std::vector<double *>{para_ex[0],para_pose[i], para_pose[j],para_depth_inv[feature_index]},std::vector<int>{1,3});
                //         marginalization_info->addResidualBlockInfo(residual_block_info);

                //     }else
                //     {
                //         StereoOneFrameResidualInfo *f = new StereoOneFrameResidualInfo(pt_i, pt_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.right_velocity,
                //                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td, CAM0_T_CAM1);
                //         ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function, std::vector<double *>{para_depth_inv[feature_index]}, std::vector<int>{0});
                //         marginalization_info->addResidualBlockInfo(residual_block_info);
                //     }
                    
                // }
            }
        }
        //printf("adding margin feature\n");

        marginalization_info->preMarginalize();

        marginalization_info->marginalize();

        //printf("revise pose address\n");
        std::unordered_map<long, double*> addr_shift;
        for(int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_pose[i])] = para_pose[i-1];
            //addr_shift[reinterpret_cast<long>(para_bias[i])] = para_bias[i-1];

        }

        addr_shift[reinterpret_cast<long>(para_ex[0])] = para_ex[0];

        std::vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if(last_marginalization_info)
        {
            delete last_marginalization_info;
        }

        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
    }else
    {
        printf("margin the second new one");
        if(last_marginalization_info && std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks),para_pose[WINDOW_SIZE-1]))
        {
            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            matrix2Double();

            if(last_marginalization_info && last_marginalization_info->valid)
            {
                std::vector<int> drop_set;
                for(int i =0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    if(last_marginalization_parameter_blocks[i] == para_pose[WINDOW_SIZE-1])
                    {
                        drop_set.push_back(i);
                    }
                }

                Marginalization *marginalization = new Marginalization(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization, NULL, last_marginalization_parameter_blocks, drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            marginalization_info->preMarginalize();

            marginalization_info->marginalize();

            std::unordered_map<long, double *> addr_shift;
            for(int i =0; i <= WINDOW_SIZE; i++)
            {
                if(i == WINDOW_SIZE-1)
                {
                     continue;
                }else if(i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_pose[i])] = para_pose[i-1];
                    //addr_shift[reinterpret_cast<long>(para_bias[i])] = para_bias[i-1];
                }else
                {
                    addr_shift[reinterpret_cast<long>(para_pose[i])] = para_pose[i];
                    //addr_shift[reinterpret_cast<long>(para_bias[i])] = para_bias[i];
                }
                
            }

            addr_shift[reinterpret_cast<long>(para_ex[0])] = para_ex[0];

            std::vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if(last_marginalization_info)
            {
                delete last_marginalization_info;
            }

            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
        }
    }
}

void Estimator::check()
{
    for(int i =0; i < frame_count; ++i)
    {
        Eigen::Matrix3d L0_Ri = all_image_frame[i].second.L0_R;
        Eigen::Vector3d L0_Pi = all_image_frame[i].second.L0_T;//-all_image_frame[0].second.L0_T;

        Eigen::Vector3d Pi = TLC[0].block<3,3>(0,0) * Ps[i] + TLC[0].block<3,1>(0,3);

        // std::cout << "Pi: " << Pi.transpose() << std::endl;
        // std::cout << "L0 Pi: " << L0_Pi.transpose() << std::endl;

        Eigen::Vector3d dp_c0 = Rs[i].transpose() * (Ps[i+1] - Ps[i]);
        Eigen::Quaterniond dq_c0 = Eigen::Quaterniond(Rs[i].transpose() * Rs[i+1]);

        int j = i+1;
        Eigen::Matrix3d L0_Rj = all_image_frame[j].second.L0_R;
        Eigen::Vector3d L0_Pj = all_image_frame[j].second.L0_T;// - all_image_frame[0].second.L0_T;

        Eigen::Vector3d dp_l0 = L0_Ri.transpose() * (L0_Pj - L0_Pi);
        Eigen::Quaterniond dq_l0 = Eigen::Quaterniond(L0_Ri.transpose() * L0_Rj);

        // std::cout << "dq: " << dq.vec().transpose() << std::endl;
        // std::cout << "dq_i: " << dq_i.vec().transpose() << std::endl;

        std::cout << "dp c0: " << dp_c0.transpose() << std::endl;
        std::cout << "dp l0: " << dp_l0.transpose() << std::endl;
    }

}




