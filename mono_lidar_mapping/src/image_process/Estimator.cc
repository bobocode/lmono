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
        mono_visual_odometer_params_.calib.cu = cameras[0].cx;
        mono_visual_odometer_params_.calib.cv = cameras[0].cy;
        mono_visual_odometer_params_.calib.f = cameras[0].fx;
        mono_visual_odometer_params_.height = CAMERA_HEIGHT;
        mono_visual_odometer_params_.pitch = CAMERA_PITCH;

        mono_visual_odometer_.reset(new viso::VisualOdometryMono(mono_visual_odometer_params_));
    }

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
    rot <<  -0.0648221, -0.00140819,  0.997896,
            -0.997858,  0.00895319,  -0.064807,
            -0.00884309,  -0.999959, -0.00198554;
    
    TLC[0].block<3,3>(0,0) = rot;
    ESTIMATE_LASER = 1;

    MonoResidualInfo::sqrt_info = FACTOR_WEIGHT * Eigen::Matrix2d::Identity();
    LASERFactor::sqrt_info = LASER_W * FACTOR_WEIGHT * Eigen::Matrix<double,6,6>::Identity();
}

void setupRos(ros::NodeHandle &nh)
{
    MeasurementManager::setupRos(nh);
    PointMapping::SetupRos(nh, false);
}

double Estimator::reprojectionError(Eigen::Matrix3d &Ri, Eigen::Vector3d &Pi, Eigen::Matrix3d &Rj,Eigen::Vector3d &Pj, Eigen::Vector2d &uvi, Eigen::Vector2d &uvj, double depth, Eigen::Matrix4d &TLC0, Eigen::Matrix4d cam0_t_cam1)
{
    Eigen::Vector3d pt_i, pt_j;

    pt_i.head<2>() = uvi;
    pt_i.z() = 1.0;

    pt_j.head<2>() = uvj;
    pt_j.z() = 1.0;

    Eigen::Vector3d tc0c1 = cam0_t_cam1.block<3,1>(0,3);
    Eigen::Matrix3d Rc0c1 = cam0_t_cam1.block<3,3>(0,0);

    Eigen::Matrix3d Rlc0 = TLC0.block<3,3>(0,0);
    Eigen::Vector3d tlc0 = TLC0.block<3,1>(0,3);

    Eigen::Vector3d pt_c0_i = depth * pt_i;
    Eigen::Vector3d pt_l_i = Rlc0 * pt_c0_i + tlc0;

    Eigen::Vector3d pt_w = Ri * pt_l_i + Pi;
    Eigen::Vector3d pt_l_j = Rj.transpose() * (pt_w - Pj);

    Eigen::Vector3d pt_c0_j = Rlc0.transpose() *(pt_l_j - tlc0);

    Eigen::Vector3d pt_c1_j = Rc0c1.transpose() *(pt_c0_j - tc0c1);

    //std::cout << "pt ci j: " << (pt_c1_j / pt_c1_j.z()).transpose() << std::endl;
    //std::cout << "pt j: " << pt_j.transpose() << std::endl;

    Eigen::Vector2d residual = (pt_c1_j / pt_c1_j.z()).head<2>() - pt_j.head<2>();
    
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
    //printf("sum reprojection error: %f\n", sum_err/count);
    //printf("outlier Num: %d\n", rejectCnt);
}

void Estimator::processCompactData(const sensor_msgs::PointCloud2ConstPtr &compact_data,
                                   const std_msgs::Header &header) 
{
    //1. process laser data
    PointMapping::CompactDataHandler(compact_data);
    Eigen::Transform transform_to_init_ = transform_aft_mapped_;
}

cv::Mat Estimator::getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}

bool Estimator::visoHandler(cv::Mat &img0, Eigen::Matrix4d &pos, cv::Mat &img1)
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

    bool success = mono_visual_odometer_->process(l_image_data, dims);

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

    }

    return success;
    
}


void Estimator::processImage(const double &header,cv::Mat &img0, Eigen::Matrix4d &transform_to_init,cv::Mat &img1)
{
    ROS_INFO_STREAM(">>>>>>>>>>>>>>>>>>>new image coming<<<<<<<<<<<<<<<<<<");
    std::pair<double,std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 6, 1>>>>> feature;

    image_l = img0.clone();

    feature = feature_tracker.trackImage(header, image_l);
    if(feature_manager.featureCheck(frame_count, feature.second, header))
    {
        marginalization_flag = MARGIN_OLD;
    }else
    {
        marginalization_flag = MARGIN_SECOND_NEW;
    }

    Eigen::Matrix4d pos;
    pos.setIdentity();

    if(OPEN_VISO)
    {
        if(visoHandler(image_l, pos,cv::Mat()))
        {
            C0_Pos = prev_cam_pose * pos;
        }
    }

    Header[frame_count] = header;
    ImageFrame imageframe(feature.second, header,transform_to_init, C0_Pos);
    all_image_frame.push_back(std::make_pair(header, imageframe));

    if(ESTIMATE_LASER == 2)
    {
        Eigen::Matrix4d laser_delta_pose = prev_laser_pose.inverse() * transform_to_init;
        Eigen::Quaterniond laser_delta_q = Eigen::Quaterniond(laser_delta_pose.block<3,3>(0,0));
        

        if(frame_count !=0 && !OPEN_VISO)
        {
            std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> corres = feature_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_rcl;
            Eigen::Quaterniond camera_delta_q = Eigen::Quaterniond(axxbsolver.solveRelativeR(corres));

            if(axxbsolver.CalibrationExRotation(camera_delta_q,laser_delta_q, calib_rcl, 10))
            {
                ROS_WARN_STREAM("initial extrinsic rotation calib between cam0 and laser success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_rcl);

                TLC[0].block<3,3>(0,0) = calib_rcl.transpose();
                TLC[1].block<3,3>(0,0) = calib_rcl.transpose();

                ESTIMATE_LASER = 1;
            }

        }

        if(frame_count !=0 && OPEN_VISO)
        {
            Matrix3d calib_rcl;
            Eigen::Quaterniond camera_delta_q = Eigen::Quaterniond(pos.block<3,3>(0,0));

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
                    optimization();
                    stage_flag = INITED;
                    std::set<int> removeIndex;
                    outliersRejection(removeIndex,5);
                    feature_manager.removeOutlier(removeIndex);
                    
                    ROS_INFO("Initialization finish!");
                    slideWindow();
                }else
                {
                    slideWindow();
                }
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
        outliersRejection(removeIndex,100.0);
        //feature_manager.removeOutlier(removeIndex);
        //check();
        optimization();
        //check();
        //ROS_WARN_STREAM("removing outliers");
        outliersRejection(removeIndex,5);
        feature_manager.removeOutlier(removeIndex);
        slideWindow();
        feature_manager.removeFailures();

    }

    prev_time = header;
    prev_cam_pose = C0_Pos;
}

void Estimator::processEstimation()
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
        sensor_msgs::PointCloud2ConstPtr pts_msg = measurement.second;
        ROS_INFO_STREAM("processing laser data with stamp " << pts_msg->header.stamp.toSec());
        //this->processCompactData(pts_msg, pts_msg->header);

        PointMapping::CompactDataHandler(pts_msg);

        L0_Q.x() = transform_aft_mapped_.rot.x();
        L0_Q.y() = transform_aft_mapped_.rot.y();
        L0_Q.z() = transform_aft_mapped_.rot.z();
        L0_Q.w() = transform_aft_mapped_.rot.w();

        L0_P.x() = transform_aft_mapped_.pos.x();
        L0_P.y() = transform_aft_mapped_.pos.y();
        L0_P.z() = transform_aft_mapped_.pos.z();

        L0_Pos.block<3,3>(0,0) = L0_Q.toRotationMatrix();
        L0_Pos.block<3,1>(0,3) = L0_P;

        sensor_msgs::ImageConstPtr img0_msg = measurement.first;
        ROS_INFO_STREAM('processing image dara with stamp ' << img0_msg->header.stamp.toSec());
        image_l = getImageFromMsg(img0_msg);

        processImage(img0_msg->header.stamp.toSec(), image_l, L0_Pos, cv::Mat());
        prev_laser_pose = L0_Pos;
    }

    thread_mutex_.unlock();
    buf_mutex_.lock();
    state_mutex_.lock();

    state_mutex_.unlock();
    buf_mutex_.unlock();
    
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
    if(stage_flag != INITED)
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

        tmp_A.block<3,3>(0,0) = -(laser_Rij.normalized() - Eigen::Matrix3d::Identity());
        tmp_A.block<3,1>(0,3) = rlc * cam_Pij;

        tmp_b.block<3,1>(0,0) = laser_Pij;

        A += tmp_A.transpose() * tmp_A;
        b += tmp_A.transpose() * tmp_b;
    }

    x = A.ldlt().solve(b);
    tlc = x.segment<3>(0);
    double s = x(3);
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
        for(int i =0; i < frame_count+1; i++)
        {
            cv::Mat r, rvec, t, D, tmp_r;

            if(all_image_frame[i].first == Header[i])
            {
                all_image_frame[i].second.R = Q[i].toRotationMatrix();
                all_image_frame[i].second.T = T[i];
                continue;
            }
        }
    }

    Eigen::VectorXd x;

    if(!solveInitialEx(all_image_frame,x))
    {
        return false;
    }

    double s = x(3);

    Eigen::Matrix3d rlc = TLC[0].block<3,3>(0,0);
    Eigen::Vector3d tlc = TLC[0].block<3,1>(0,3);

    Eigen::Matrix3d cam_R0 = all_image_frame[0].second.R;
    Eigen::Vector3d cam_P0 = all_image_frame[0].second.T;

    for(int i =0; i <=frame_count;i++)
    {
        Eigen::Matrix3d cam_Ri = all_image_frame[i].second.R;
        Eigen::Vector3d cam_Pi = all_image_frame[i].second.T;

        Rs[i] = cam_Ri * rlc.transpose();
        Ps[i] = s * cam_Pi - Rs[i] * tlc - (s * cam_P0 - cam_R0 * rlc.transpose() * tlc);

        Eigen::Matrix3d L0_Ri = all_image_frame[i].second.L0_R;
        Eigen::Vector3d L0_Pi = all_image_frame[i].second.L0_T;

        std::cout << "Rs[" << i << "]\n" << Rs[i] << std::endl;
        std::cout << "Ps[" <<i <<"]" << Ps[i].transpose() << std::endl;

        //std::cout << "L0_R[" << i << "]\n" << L0_Ri << std::endl;
        //std::cout << "L0_P[" << i << "]\n" << L0_Pi.transpose() << std::endl;
    }

    feature_manager.clearDepth();
    feature_manager.triangulate(frame_count,Rs,Ps,TLC[0]);

    std::set<int> removeIndex;
    outliersRejection(removeIndex, 100.0);
    feature_manager.removeOutlier(removeIndex);
    //outliersRejection(removeIndex, 3.0);

    return true;

}

void Estimator::matrix2Double()
{
    Eigen::Matrix3d RL0_new = all_image_frame[10].second.L0_R;
    Eigen::Vector3d TL0_new = all_image_frame[10].second.L0_T;

    Eigen::Matrix3d RL0_last = all_image_frame[9].second.L0_R;
    Eigen::Vector3d TL0_last = all_image_frame[9].second.L0_T;

    Eigen::Matrix3d dR = RL0_last.transpose() * RL0_new;
    Eigen::Vector3d dp = RL0_last.transpose() * (TL0_new - TL0_last);

    Rs[10] = Rs[9] * dR;
    Ps[10] = Rs[9] * dp + Ps[9];

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

    std::cout << "feature size: " << dep.size() << std::endl;
    for(int i =0; i < feature_manager.getFeatureCount(); i++)
    {
        //printf("feature %d depth: %f\n",i,1.0/dep(i));
        para_depth_inv[i][0] = dep(i);
    }

}

void Estimator::double2Matrix()
{
    //Eigen::Vector3d origin_R0 = Utility::R2ypr(laser_pose[0].block<3,3>(0,0));
    Eigen::Vector3d origin_R0 = mathutils::R2ypr(Rs[0]);
    //Eigen::Vector3d origin_t0 = laser_pose[0].block<3,1>(0,3);
    Eigen::Vector3d origin_t0 = Ps[0];

    Eigen::Vector3d origin_R00 = mathutils::R2ypr(Eigen::Quaterniond(para_pose[0][6], para_pose[0][3], para_pose[0][4], para_pose[0][5]).toRotationMatrix());
    double y_diff = origin_R0.x() - origin_R00.x();

    std::cout << "rot diff: " << (origin_R0 - origin_R00).transpose() << std::endl;

    Eigen::Matrix3d rot_diff = mathutils::ypr2R(Eigen::Vector3d(y_diff, 0, 0));


    for(int i=0; i <= WINDOW_SIZE; i++)
    {
        Eigen::Vector3d t_i(para_pose[i][0]-para_pose[0][0], para_pose[i][1]-para_pose[0][1], para_pose[i][2]-para_pose[0][2]);
        Eigen::Quaterniond q_i(para_pose[i][6], para_pose[i][3], para_pose[i][4], para_pose[i][5]);

        //laser_pose[i].block<3,1>(0,3) =  rot_diff * t_i + origin_t0;
        //laser_pose[i].block<3,3>(0,0) =  rot_diff * q_i.normalized().toRotationMatrix();

        //ROS_INFO_STREAM("Ps[" << i << "] before: " << (Ps[i]).transpose());
        Ps[i] = rot_diff * t_i + origin_t0;
        Rs[i] = rot_diff * q_i.normalized().toRotationMatrix();
        //ROS_INFO_STREAM("Ps[" << i << "] after: " << Ps[i].transpose());
    }

    Eigen::Quaterniond q_10(Rs[10]);

    // fprintf(odometry, "%d,%f,%f,%f , %f, %f, %f, %f \n", inputImageCnt,Ps[10].x() , Ps[10].y(),Ps[10].z(),
    //                                                             q_10.w(), q_10.x(), q_10.y(), q_10.z());
    // fflush(odometry);

    Eigen::Vector3d t_ex1(para_ex[0][0], para_ex[0][1], para_ex[0][2]);
    Eigen::Quaterniond q_ex1(para_ex[0][6], para_ex[0][3], para_ex[0][4], para_ex[0][5]);

    // fprintf(extrinsic, "%d,%f,%f,%f , %f, %f, %f, %f \n", inputImageCnt,t_ex1.x() ,t_ex1.y(),t_ex1.z(),
    //                                                             q_ex1.w(), q_ex1.x(), q_ex1.y(), q_ex1.z());
    // fflush(extrinsic);

    TLC[0].block<3,1>(0,3) = t_ex1;
    TLC[0].block<3,3>(0,0) = q_ex1.toRotationMatrix();

    ROS_WARN_STREAM("extrinsic parameters: Rlc" << endl << TLC[0].inverse());
    //ROS_WARN_STREAM("extrinsic parameters: tlc " << t_ex1.transpose());

    Eigen::VectorXd dep = feature_manager.getDepthVector();
    for(int i =0; i < feature_manager.getFeatureCount(); i++)
    {   
        //ROS_INFO_STREAM("depth before: " << 1.0/ dep(i));
        dep(i) = para_depth_inv[i][0];
        //ROS_INFO_STREAM("depth after: " << 1.0/ dep(i));
    }

    feature_manager.setDepth(dep);

}







