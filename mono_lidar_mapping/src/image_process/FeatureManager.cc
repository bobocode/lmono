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

adapted from VINS-mono
*******************************************************/
#include "image_process/Feature_Manager.h"

FeaturePerId::~FeaturePerId(){}

int FeaturePerId::endFrame()
{
    return start_frame + feature_per_frame.size()-1;
}

FeatureManager::FeatureManager(){}

FeatureManager::~FeatureManager(){}

void FeatureManager::clearDepth()
{
    for (auto &it_per_id : feature)
    {
        it_per_id.solve_flag = 0;
        it_per_id.estimated_depth = INIT_DEPTH;
    }
}

void FeatureManager::setDepth(const Eigen::VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        
        if (it_per_id.used_num < 4)
            continue;

        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        //ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        if (it_per_id.estimated_depth < 0.1)
        {
            it_per_id.solve_flag = 2;
        }
        else
            it_per_id.solve_flag = 1;
    }
}

Eigen::VectorXd FeatureManager::getDepthVector()
{
    Eigen::VectorXd dep_vec(getFeatureCount());
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
    
        if (it_per_id.used_num < 4)
            continue;

        dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
    }
    return dep_vec;
}

void FeatureManager::triangulate(int frameCnt,  Eigen::Matrix3d Rs[], Eigen::Vector3d Ps[], Eigen::Matrix4d tlc)
{    
    std::cout << "features: " << feature.size() << std::endl; 
    //std::cout << "extrinsic parameters: \n" << tlc.matrix() << std::endl; 

    for(auto &it_per_id: feature)
    {
        if(it_per_id.estimated_depth > 0)
        {
            continue;
        }
 
        /*if(it_per_id.feature_per_frame.size() >1)
        {
            int i = it_per_id.start_frame;
            Eigen::Matrix<double, 3, 4> pose0;

            Eigen::Vector3d t0 = Ps[i] + Rs[i] * tlc.block<3,1>(0,3);
            Eigen::Matrix3d R0 = Rs[i] * tlc.block<3,3>(0,0);
            pose0.leftCols<3>() = R0.transpose();
            pose0.rightCols<1>() = -R0.transpose() * t0;

            i++;

            Eigen::Matrix<double, 3, 4> pose1;

            Eigen::Vector3d t1 = Ps[i] + Rs[i] * tlc.block<3,1>(0,3);
            Eigen::Matrix3d R1 = Rs[i] * tlc.block<3,3>(0,0);
            pose1.leftCols<3>() = R1.transpose();
            pose1.rightCols<1>() = -R1.transpose() * t1;

            Eigen::Vector2d point0, point1;
            Eigen::Vector3d point3d;

            point0 = it_per_id.feature_per_frame[0].pt;
            point1 = it_per_id.feature_per_frame[1].pt;

            triangulatePoint(pose0, pose1, point0, point1, point3d);
            Eigen::Vector3d localPoint;
            localPoint = pose0.leftCols<3>() * point3d + pose0.rightCols<1>();

            //printf("stereo feature %d depth: %f\n", it_per_id.feature_id, it_per_id.estimated_depth);
            //printf("two frame feature %d depth: %f\n", it_per_id.feature_id, localPoint.z());

            if(localPoint.z() > 0)
            {
                it_per_id.estimated_depth = localPoint.z();
                continue;

            }else
            {
                it_per_id.estimated_depth = INIT_DEPTH;
            }

        }*/

        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if(it_per_id.used_num <4)
        {
            continue;
        }

        int i = it_per_id.start_frame, j = i-1;
        
        Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
        int svd_idx = 0;

        Eigen::Vector3d t0 = Ps[i] + Rs[i] * tlc.block<3,1>(0,3);
        Eigen::Matrix3d R0 = Rs[i] * tlc.block<3,3>(0,0);
        
        for(auto &it_per_frame: it_per_id.feature_per_frame)
        {
            j++;

            Eigen::Vector3d t1 = Ps[j] + Rs[j] * tlc.block<3,1>(0,3);
            Eigen::Matrix3d R1 = Rs[j] * tlc.block<3,3>(0,0);

            Eigen::Vector3d t = R0.transpose() *(t1-t0);
            Eigen::Matrix3d R = R0.transpose() * R1;

            Eigen::Matrix<double,3,4> P;
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose()*t;
            
            Eigen::Vector3d f;
            f.x() = it_per_frame.pt.x();
            f.y() = it_per_frame.pt.y();
            f.z() = 1.0;
            f = f.normalized();
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            if (i == j)
                continue;
        }

        if(svd_idx !=svd_A.rows())
        {
            std::cout << "svd range wrong" << std::endl;
            abort();
        }

        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        Eigen::Vector3d localPoint;
        localPoint = svd_V.head<3>()/svd_V[3];
        
        //it_per_id->estimated_depth = -b / A;
        //it_per_id->estimated_depth = svd_V[2] / svd_V[3];
        //printf("stereo feature %d depth: %f\n", it_per_id.feature_id, it_per_id.estimated_depth);
        //printf("Multi frames feature %d depth: %f\n", it_per_id.feature_id, localPoint.z());

        if(localPoint.z() < 0.1)
        {
            it_per_id.estimated_depth = INIT_DEPTH;

        }else{
            
            it_per_id.estimated_depth = localPoint.z();
        }

        // if(it_per_id.feature_per_frame[0].stereo)
        // {
        //     cv::Point3d point;
        //     cv::Point2d rect_uv;

        //     rect_uv.x = it_per_id.feature_per_frame[0].pt.x();
        //     rect_uv.y = it_per_id.feature_per_frame[0].pt.y();

        //     double disparity = it_per_id.feature_per_frame[0].uv.x() - it_per_id.feature_per_frame[0].right_uv.x();
        //     stereo_model.projectDisparityTo3d(rect_uv, disparity,point);

        //     if(point.z >0)
        //     {
        //         it_per_id.estimated_depth = point.z;
        //         //printf("stereo feature %d depth: %f\n", it_per_id.feature_id, point.z);
        //         continue;
        //     }

        // }

    }
       
}

void FeatureManager::clearState()
{
    feature.clear();
}

int FeatureManager::getFeatureCount()
{
    int cnt=-1;
    for(auto &it: feature)
    {
        it.used_num = it.feature_per_frame.size();

        if (it.used_num < 4)
            continue;

        ++cnt;
    }

    return cnt+1;

}

double FeatureManager::computeParallax(const FeaturePerId &it_per_id, int frame_count)
{
    //check the new frame is a keyframe or not 

    //prev frame
    const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 -it_per_id.start_frame];

    //ROS_INFO_STREAM("frame i: " << frame_i.pt.transpose());

    //new coming frame
    const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count  -1 -it_per_id.start_frame];
    //ROS_INFO_STREAM("frame j: " << frame_j.pt.transpose());

    double ans=0;

    Eigen::Vector3d p_i;
    Eigen::Vector3d p_j;

    p_i.x() = frame_i.uv.x();
    p_i.y() = frame_i.uv.y();
    p_i.z() = 1.0;

    p_j.x() = frame_j.uv.x();
    p_j.y() = frame_j.uv.y();
    p_j.z() = 1.0;

    double du = p_i(0)-p_j(0);
    double dv = p_i(1)-p_j(1);

    ans = max(ans, sqrt(du * du + dv * dv));

    return ans;

}

bool FeatureManager::featureCheck(int frame_count, const std::map<int,std::vector<std::pair<int, Eigen::Matrix<double,6,1>>>> &image, double td)
{   
    //printf("adding feature...\n");
    double parallax_sum=0;
    int parallax_num =0;
    last_track_num =0;
    long_track_num = 0;
    new_feature_num =0;
    for(auto &id_pts: image)
    {
        FeaturePerFrame f_per_fra(id_pts.second[0].second, td);

        // std::cout << "feature 2d normal: " << f_per_fra.pt.transpose() << std::endl;
        // std::cout  << "feature 2d uv: " << f_per_fra.uv.transpose() << std::endl;

        if(id_pts.second.size()==2)
        {
            f_per_fra.rightFrame(id_pts.second[1].second);
        }

        int feature_id = id_pts.first;
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
        {
            return it.feature_id == feature_id;
        });

        m.lock();
        if(it==feature.end())
        {
            feature.push_back(FeaturePerId(feature_id, frame_count));
            feature.back().feature_per_frame.push_back(f_per_fra);
            feature.back().start_frame = frame_count;
            new_feature_num++;

        }else if(it->feature_id == feature_id)
        {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;

            if(it->feature_per_frame.size() >=4)
            {
                long_track_num++;
            }
        }
        m.unlock();
    }

    //printf("long track num: %d\n", long_track_num);
    //printf("last track num: %d\n", last_track_num);
    //printf("new feature num: %d\n", new_feature_num);

    if(frame_count <2 || last_track_num <20 || long_track_num < 40 ||new_feature_num > 0.5 * last_track_num)
    {
        return true;
    }

    for(auto &it_per_id: feature)
    {
        int used_num = it_per_id.feature_per_frame.size();
        //printf("used num: %d\n", used_num);
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count-1)
        {
            parallax_sum += computeParallax(it_per_id,frame_count);
            parallax_num++;
        }
    }

    if(parallax_num ==0)
    {
        return true;

    }else
    {
        //printf("parallax_sum: %lf, parallax_num: %d \n", parallax_sum, parallax_num);
        //printf("current parallax: %lf \n",  parallax_sum / parallax_num);

        return parallax_sum / parallax_num >= FEATURE_THRESHOLD;
    }
    
}

void FeatureManager::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1, Eigen::Vector2d &point1, Eigen::Vector2d &point2, Eigen::Vector3d &point_3d)
{
    Eigen::Matrix<double,3,4> prev_pose;
    Eigen::Matrix<double,3,4> cur_pose;
    Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();

    prev_pose = Pose0.block<3,4>(0,0);
    cur_pose = Pose1.block<3,4>(0,0);

    design_matrix.row(0) = point1[0] * prev_pose.row(2) - prev_pose.row(0);
    design_matrix.row(1) = (point1[1] * prev_pose.row(2) - prev_pose.row(1));

    design_matrix.row(2) = point2[0] * cur_pose.row(2) - cur_pose.row(0);
    design_matrix.row(3) = (point2[1]* cur_pose.row(2) - cur_pose.row(1));

    Eigen::Vector4d triangualted_point;
    triangualted_point = design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();

    point_3d(0) = triangualted_point(0) / triangualted_point(3);
    point_3d(1) = triangualted_point(0) / triangualted_point(3);
    point_3d(2) = triangualted_point(0) / triangualted_point(3);
}

/*void FeatureManager::triangulateFrames(int frame0, Eigen::Matrix4d &Pose0, int frame1,Eigen::Matrix4d &Pose1, std::vector<FeatureStruc> &sfm)
{
    if(frame0 == frame1)
    {
        std::cerr << "Triangulation betwwen two frames cannot done" << std::endl;
        abort();
    }

    for(int i =0; i < sfm.size(); i++)
    {
        if(sfm[i].state)
        {
            continue;
        }
        bool has_1=false;
        bool has_2 = false;

        Eigen::Vector2d point0;
        Eigen::Vector2d point1;

        for(int k =0; k < sfm[i].tracked.size(); k++)
        {
            if(sfm[i].tracked[k].first==frame0)
            {
                point0 = sfm[i].tracked[k].second;
                has_1=true;
            }

            if(sfm[i].tracked[k].first==frame1)
            {
                point1 = sfm[i].tracked[k].second;
                has_2=true;
            }
        }

        if(has_1 && has_2)
        {
            Eigen::Vector3d point_3d;
            triangulatePoint(Pose0, Pose1, point0, point1, point_3d);
            sfm[i].state=true;
            sfm[i].CamPosition = point_3d;
        }
    }
}*/

void FeatureManager::removeFailures()
{
    for (auto it = feature.begin(), it_next = feature.begin();
        it !=feature.end(); it = it_next)
        {
            it_next++;
            
            if(it->solve_flag == 2)
            {
                feature.erase(it);
            }
        }
}

void FeatureManager::removeOutlier(std::set<int> &outlierIndex)
{
    std::set<int>::iterator itSet;
    for(auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next)
    {
        it_next++;

        int index = it->feature_id;
        itSet = outlierIndex.find(index);

        if(itSet != outlierIndex.end())
        {
            feature.erase(it);
        }
    }

}

void FeatureManager::removeBack()
{
    for(auto it=feature.begin(), it_next = feature.begin();it !=feature.end(); it = it_next)
    {
        it_next++;
        if(it->start_frame !=0)
            it->start_frame--;
        else
        {
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if(it->feature_per_frame.size()==0)
                feature.erase(it);
        }
    } 
}

void FeatureManager::removeFront(int frame_count)
{
    for(auto it = feature.begin(), it_next = feature.begin(); it !=feature.end(); it=it_next)
    {
        it_next++;
        if(it->start_frame ==frame_count)
        {
            it->start_frame--;
        }else
        {
            int j = WINDOW_SIZE -1 - it->start_frame;
            if(it->endFrame()< frame_count-1)
            {
                continue;
            }
                
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
            if(it->feature_per_frame.size()==0)
            {
                feature.erase(it);
            }
                
        }
        
    }
}

void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P)
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
        {
            it->start_frame--;
        }else
        {
            //printf("starting frame: %d\n", it->start_frame);
            Eigen::Vector3d uv_i;
            uv_i.x() = it->feature_per_frame[0].pt.x(); 
            uv_i.y() = it->feature_per_frame[0].pt.y();
            uv_i.z() = 1.0;

            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() < 2)
            {
                feature.erase(it);
                continue;
            }
            else
            {   
                Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
                double dep_j = pts_j(2);
                //printf("depth at marg frame: %f\n", it->estimated_depth);
                //printf("depth after marg %f\n", dep_j);
                if (dep_j > 0)
                {
                    it->estimated_depth = dep_j;
                }else
                {
                    it->estimated_depth = INIT_DEPTH;
                }
                
            }
        }
        // remove tracking-lost feature after marginalize
        /*
        if (it->endFrame() < WINDOW_SIZE - 1)
        {
            feature.erase(it);
        }
        */
    }
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> corres;
    //std::cout << "feature size: " << feature.size() << std::endl;
    for(auto &it: feature)
    {   
        //std::cout << "starting frame: " << it.start_frame << std::endl;
        //std::cout << " frame count: " << frame_count_l << std::endl;
        //std::cout << "end frame: " << it.endFrame() << std::endl;
        if(it.start_frame <= frame_count_l && it.endFrame() >=frame_count_r)
        {
            Eigen::Vector3d a = Eigen::Vector3d::Zero();
            Eigen::Vector3d b = Eigen::Vector3d::Zero();

            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;

            a.head<2>() = it.feature_per_frame[idx_l].pt;
            b.head<2>() = it.feature_per_frame[idx_r].pt;

            a.z() = 1.0;
            b.z() = 1.0;

            corres.push_back(std::make_pair(a,b));
        }
    }

    return corres;
}

bool FeatureManager::initialStructure(std::vector<std::pair<double, ImageFrame>> &all_image_frame)
{
    return true;
}

bool FeatureManager::relativePose(Eigen::Matrix3d &relative_R, Eigen::Vector3d &relative_T, int &l)
{
    for(int i =0; i < WINDOW_SIZE; i++)
    {
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> corres;
        corres = getCorresponding(i, WINDOW_SIZE);
        if (corres.size() > 20)
        {
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++)
            {
                Eigen::Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Eigen::Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;

            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            if(average_parallax * 460 > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i;
                ROS_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure", average_parallax * 460, l);
                return true;
            }
        }
    }
    return false;

}