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

#include "image_process/Feature_Tracker.h"

FeatureTracker::FeatureTracker()
{
    n_id = 0;
    hasPrediction = false;  
    MAX_CNT = 150;
    frame_count= 0;
    inputFrameCnt = -1;
}

FeatureTracker::~FeatureTracker(){}

bool FeatureTracker::inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < col - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < row - BORDER_SIZE;
}


void FeatureTracker::reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void FeatureTracker::reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void FeatureTracker::setMask()
{
    mask = cv::Mat(row, col, CV_8UC1, cv::Scalar(255));

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < cur_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(cur_pts[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    cur_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            cur_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

double FeatureTracker::distance(cv::Point2f &pt1, cv::Point2f &pt2)
{
    //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}

std::vector<cv::Point2f> FeatureTracker::ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                    map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts)
{
    //printf("computing velocity...\n");
    vector<cv::Point2f> pts_velocity;
    cur_id_pts.clear();
    for (unsigned int i = 0; i < ids.size(); i++)
    {
        cur_id_pts.insert(make_pair(ids[i], pts[i]));
    }

    // caculate points velocity
    if (!prev_id_pts.empty())
    {
        double dt = cur_time - prev_time;
        
        for (unsigned int i = 0; i < pts.size(); i++)
        {
            std::map<int, cv::Point2f>::iterator it;
            it = prev_id_pts.find(ids[i]);
            if (it != prev_id_pts.end())
            {
                double v_x = (pts[i].x - it->second.x) / dt;
                double v_y = (pts[i].y - it->second.y) / dt;
                pts_velocity.push_back(cv::Point2f(v_x, v_y));
            }
            else
                pts_velocity.push_back(cv::Point2f(0, 0));

        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    return pts_velocity;
}

void FeatureTracker::drawTrack(const cv::Mat &imLeft, cv::Mat &imRight,
                                   vector<int> &curLeftIds,
                                   vector<cv::Point2f> &curLeftPts, 
                                   map<int, cv::Point2f> &prevLeftPtsMap)
{
    cv::cvtColor(imLeft, imTrack, CV_GRAY2BGR);
    
    //cv::cvtColor(img, img, CV_GRAY2RGB);

    for (size_t j = 0; j < curLeftIds.size(); j++)
    {
        double len = std::min(1.0, 1.0 * track_cnt[j] / 20);
        cv::circle(imTrack, curLeftPts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 3);
    }

    std::map<int, cv::Point2f>::iterator mapIt;
    for(size_t i =0; i < curLeftIds.size();i++)
    {
        int id = curLeftIds[i];
        mapIt = prevLeftPtsMap.find(id);
        if(mapIt != prevLeftPtsMap.end())
        {
            cv::arrowedLine(imTrack, curLeftPts[i], mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
        }
    }
    
    std::string save_path = "/home/bo/auto-calib/check/" + to_string(cur_time) + ".jpg";
    cv::imwrite(save_path, imTrack);

    //cv::imshow("tracking", imTrack);
    //cv::waitKey(2);
}
cv::Mat FeatureTracker::getTrackImage()
{
    return imTrack;
}

vector<cv::Point2f> FeatureTracker::undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam)
{
    //printf("undistorting...\n");
    std::vector<cv::Point2f> un_pts;
    for(size_t i =0; i < pts.size(); i++)
    {
        Eigen::Vector2d a(pts[i].x, pts[i].y);
        Eigen::Vector3d b;
        cam->liftProjective(a,b);
        un_pts.push_back(cv::Point2f(b.x()/b.z(), b.y()/b.z()));
        //printf("normallized pts: %f, %f\n", b.x()/b.z(), b.y()/b.z());
    }

    return un_pts;

}

 map<int, vector<pair<int, Eigen::Matrix<double, 6, 1>>>> FeatureTracker::trackImage(double _cur_time, const cv::Mat &image_l, const cv::Mat &image_r)
 {
    //printf("tracking..%f\n", _cur_time)
    cv::Mat tmp_img;
    cv::cvtColor(image_l, tmp_img,cv::COLOR_BGR2GRAY,1);
    cur_img = tmp_img.clone();

    //printf("tracking..%f\n", _cur_time);
    cur_time = _cur_time;

    if(image_l.empty())
    {
        printf("image l is empty.\n");
    }

    row = cur_img.rows;
    col = cur_img.cols;

    cv::Mat r_img = image_r;

    cur_pts.clear();
    cur_un_pts.clear();
    cur_un_pts_map.clear();

    if(prev_pts.size() > 0)
    {
        vector<uchar> status;
        vector<float> err;

        cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21,21),3);

        vector<uchar> reverse_status;
        vector<cv::Point2f> reverse_pts = prev_pts;

        cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 1, 
        cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
    
        for(size_t i =0; i < status.size(); i++)
        {
            if(status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i])<=0.5)
            {
                status[i] =1;
            }else
            {
                status[i] = 0;
            } 
        }

        for(int i =0; i < cur_pts.size(); i++)
        {
            if(status[i] && !inBorder(cur_pts[i]))
            {
                status[i] = 0;
            }
        }

        reduceVector(prev_pts, status);
        reduceVector(cur_pts,status);
        reduceVector(ids,status);
        reduceVector(track_cnt,status);
    }

    for(auto &n : track_cnt)
    {
        n++;
    }

    if(1)
    {
        //printf("set mask begins\n");
        setMask();

        int n_max_cnt = MAX_CNT -static_cast<int>(cur_pts.size());

        if(n_max_cnt > 0)
        {
            if(mask.empty())
            {
                cout << "mask is empty " << endl;
            }

            if(mask.type() != CV_8UC1)
            {
                cout << "mask type is wrong " << endl;
            }
            //printf("extracting features\n");
            cv::goodFeaturesToTrack(cur_img, n_pts, MAX_CNT-cur_pts.size(), 0.01, MIN_DIST, mask);

        }else
        {
            n_pts.clear();
        }

        //printf("pts size %d\n", static_cast<int>(n_pts.size()));

        int idx = 0;
        for(auto &p : n_pts)
        {   
            //printf("point (%f,%f)\n", p.x,p.y);
            cur_pts.push_back(p);
            ids.push_back(n_id++);
            track_cnt.push_back(1);
            //std::cout << idx << std::endl;
            //idx++;
        }

        //printf("cur pts size %d\n", static_cast<int>(cur_pts.size()));
        
    }

    if(!r_img.empty())
    {
        ids_right.clear();
        cur_right_pts.clear();
        cur_un_right_pts.clear();

        right_pts_velocity.clear();
        cur_un_right_pts_map.clear();

        if(!cur_pts.empty())
        {
            vector<cv::Point2f> reverseLeftPts;
            vector<uchar> status, statusRightLeft;
            vector<float> err;

            cv::calcOpticalFlowPyrLK(cur_img, r_img, cur_pts, cur_right_pts, status, err, cv::Size(21,21),3);

            cv::calcOpticalFlowPyrLK(r_img, cur_img, cur_right_pts, reverseLeftPts, statusRightLeft, err, cv::Size(21, 21), 3);

            for(size_t i=0; i< status.size(); i++)
            {
                if(status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i]) && distance(cur_pts[i], reverseLeftPts[i])<=0.5)
                {
                    status[i] =1;
                }else
                {
                    status[i] = 0;
                }
                
            }

            ids_right = ids;
            //reduceVector(cur_pts, status);
            reduceVector(cur_right_pts, status);
            //reduceVector(ids, status);
            reduceVector(ids_right,status);

            cur_un_right_pts = undistortedPts(cur_right_pts, cams[0]);
            right_pts_velocity = ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map, prev_un_right_pts_map);
        }

        prev_un_right_pts_map = cur_un_right_pts_map;
    }

    cur_un_pts = undistortedPts(cur_pts, cams[0]);
    pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);

    //printf("draw track...\n");
    drawTrack(cur_img, r_img, ids, cur_pts, prevLeftPtsMap);

    prev_pts.clear();
    prev_un_pts.clear();
    prev_un_pts_map.clear();

    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    prev_un_pts_map = cur_un_pts_map;
    prev_time = cur_time;


    prevLeftPtsMap.clear();
    for(int i =0; i< cur_pts.size(); i++)
    {
        prevLeftPtsMap[ids[i]] = cur_pts[i];
    }

    map<int, vector<pair<int, Eigen::Matrix<double, 6, 1>>>> featureFrame;
    for(size_t i =0; i< ids.size(); i++)
    {
        int feature_id = ids[i];
        double x, y, z;

        x = cur_un_pts[i].x;
        y = cur_un_pts[i].y;
        z =1.0;

        double p_u, p_v;
        p_u = cur_pts[i].x;
        p_v = cur_pts[i].y;

        int camera_id =0;
        double velocity_x, velocity_y;
        velocity_x = pts_velocity[i].x;
        velocity_y = pts_velocity[i].y;

        Eigen::Matrix<double, 6,1> xyz_uv_velocity;
        xyz_uv_velocity << x,y, p_u, p_v, velocity_x, velocity_y;

        //std::cout << "left feature frame: " << xyz_uv_velocity.transpose() << std::endl;

        featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
    }

    if(!r_img.empty())
    {
        for(size_t i =0; i < ids_right.size(); i++)
        {
            int feature_id = ids_right[i];
            double x, y, z;
            x = cur_un_right_pts[i].x;
            y = cur_un_right_pts[i].y;
            z = 1.0;
            //std::cout << "right pt: " << x << " " << y << std::endl;

            double p_u, p_v;
            p_u = cur_right_pts[i].x;
            p_v = cur_right_pts[i].y;

            //std::cout << "right uv: " << p_u << " " << p_v << std::endl;

            int camera_id =1;
            double velocity_x, velocity_y;
            velocity_x = right_pts_velocity[i].x;
            velocity_y = right_pts_velocity[i].y;

            Eigen::Matrix<double, 6,1> xyz_uv_velocity;
            xyz_uv_velocity << x,y ,p_u,p_v, velocity_x, velocity_y;

            //std::cout << "right xyz uv velocity : " << xyz_uv_velocity.transpose() << std::endl;

            featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
        }
        
        
    }

    return featureFrame;
 }