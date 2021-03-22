#include "loop_detection/Loop_Detector.h"

FILE* loop_odometry= fopen("/home/bo/MonoLidarMapping/loop_odometry.txt","w+");

LoopDetector::LoopDetector()
{
    earliest_loop_index = -1;
    t_drift = Eigen::Vector3d(0, 0, 0);
    yaw_drift = 0;
    r_drift = Eigen::Matrix3d::Identity();
    w_t_vio = Eigen::Vector3d(0, 0, 0);
    w_r_vio = Eigen::Matrix3d::Identity();
    global_index = 0;
    sequence_cnt = 0;
    sequence_loop.push_back(0);
    base_sequence = 1;
}

LoopDetector::~LoopDetector()
{
    t_optimization.join();
}

void LoopDetector::loadVocabulary(std::string voc_path)
{
    voc = new BriefVocabulary(voc_path);
    db.setVocabulary(*voc, false, 0);
}

int LoopDetector::addKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop)
{
    //shift to base frame
    // Vector3d vio_P_cur;
    // Matrix3d vio_R_cur;

    if (sequence_cnt != cur_kf->sequence)
    {
        sequence_cnt++;
        sequence_loop.push_back(0);
        w_t_vio = Eigen::Vector3d(0, 0, 0);
        w_r_vio = Eigen::Matrix3d::Identity();
        m_drift.lock();
        t_drift = Eigen::Vector3d(0, 0, 0);
        r_drift = Eigen::Matrix3d::Identity();
        m_drift.unlock();
    }

    // cur_kf->getVioPose(vio_P_cur, vio_R_cur);
    // vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
    // vio_R_cur = w_r_vio *  vio_R_cur;
    // cur_kf->updateVioPose(vio_P_cur, vio_R_cur);

    // cur_kf->index = global_index;
    // global_index++;

	int loop_index = -1;

    if (flag_detect_loop)
    {
        TicToc tmp_t;
        loop_index = detectLoop(cur_kf, cur_kf->index);
    }
    else
    {
        addKeyFrameIntoVoc(cur_kf);
    }

    if(loop_index !=-1)
    {
        KeyFrame* old_kf = getKeyFrame(loop_index);
        if(abs(cur_kf->time_stamp - old_kf->time_stamp) > LOOP_SEARCH_TIME)
        {
            //printf(YEL " %d detect loop with %d \n" WHT, cur_kf->index, loop_index);
            if(cur_kf->findConnection(old_kf))
            {
                loop_index = old_kf->index;

                Eigen::Vector3d w_P_old, w_P_cur, vio_P_cur;
                Eigen::Matrix3d w_R_old, w_R_cur, vio_R_cur;

                old_kf->getVioPose(w_P_old,w_R_old);
                cur_kf->getVioPose(vio_P_cur,vio_R_cur);

                Eigen::Vector3d relative_t;
                Eigen::Quaterniond relative_q;

                relative_t = cur_kf->getLoopRelativeT();
                relative_q = cur_kf->getLoopRelativeQ();

                w_P_cur = w_R_old * relative_t + w_P_old;
                w_R_cur = w_R_old * relative_q.toRotationMatrix();

                cur_kf->updateVioPose(w_P_cur, w_R_cur);

                // if(old_kf->sequence != cur_kf->sequence && sequence_loop[cur_kf->sequence]==0)
                // {

                //     for(; it !=keyframelist.end(); it++)
                //     {
                //         if((*it)->sequence == cur_kf->sequence)
                //         {
                //             (*it)->updateVioPose(w_P_cur, w_R_cur);
                //         }
                //     }

                //     sequence_loop[cur_kf->sequence] =1;
                // }

                // ROS_INFO_STREAM("publish correct odom");
                // Eigen::Quaterniond Q(w_R_cur);

                // nav_msgs::Odometry odometry;
                // odometry.header.stamp = ros::Time(cur_kf->time_stamp);
                // odometry.header.frame_id = "camera_init";
                // odometry.pose.pose.position.x = w_P_cur.x();
                // odometry.pose.pose.position.y = w_P_cur.y();
                // odometry.pose.pose.position.z = w_P_cur.z();

                // odometry.pose.pose.orientation.w = Q.w();
                // odometry.pose.pose.orientation.x = Q.x();
                // odometry.pose.pose.orientation.y = Q.y();
                // odometry.pose.pose.orientation.z = Q.z();

                // pub_correct_odom_.publish(odometry);

            }else
            {
                loop_index = -1;
            }
        }
    }

    Eigen::Vector3d cur_T;
    Eigen::Matrix3d cur_R;

    cur_kf->getVioPose(cur_T, cur_R);
    Eigen::Quaterniond Q(cur_R);

    fprintf(loop_odometry, "%f,%f,%f,%f , %f, %f, %f, %f \n", cur_kf->time_stamp,cur_T.x() ,cur_T.y(),cur_T.z(),
                                                                Q.w(), Q.x(), Q.y(), Q.z());
    fflush(loop_odometry);

    m_keyframelist.lock();
    keyframelist.push_back(cur_kf);
    m_keyframelist.unlock();

    return loop_index;
}

KeyFrame* LoopDetector::getKeyFrame(int index)
{
//    unique_lock<mutex> lock(m_keyframelist);
    list<KeyFrame*>::iterator it = keyframelist.begin();
    for (; it != keyframelist.end(); it++)   
    {
        if((*it)->index == index)
            break;
    }
    if (it != keyframelist.end())
        return *it;
    else
        return NULL;
}

int LoopDetector::detectLoop(KeyFrame* keyframe, int frame_index)
{
    cv::Mat compressed_image;

    if (DEBUG_IMAGE)
    {
        int feature_num = keyframe->brief_keypoints.size();
        cv::resize(keyframe->image, compressed_image, cv::Size(376, 240));
        putText(compressed_image, "feature_num:" + to_string(feature_num), cv::Point2f(10, 10), CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
        image_pool[frame_index] = compressed_image;
    }

    QueryResults ret;
    TicToc t_query;
    db.query(keyframe->brief_descriptors, ret, 4, frame_index - LOOP_SEARCH_GAP);
    printf("query time: %f\n", t_query.Toc());
    cout << "Searching for Image " << frame_index << ". " << ret << endl;

    TicToc t_add;
    db.add(keyframe->brief_descriptors);
    printf("add feature time: %f\n", t_add.Toc());
    // ret[0] is the nearest neighbour's score. threshold change with neighour score

    bool find_loop = false;
    cv::Mat loop_result;

    if (DEBUG_IMAGE)
    {
        loop_result = compressed_image.clone();
        if (ret.size() > 0)
            putText(loop_result, "neighbour score:" + to_string(ret[0].Score), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));

        for (unsigned int i = 0; i < ret.size(); i++)
        {
            int tmp_index = ret[i].Id;
            auto it = image_pool.find(tmp_index);
            cv::Mat tmp_image = (it->second).clone();
            putText(tmp_image, "index:  " + to_string(tmp_index) + "loop score:" + to_string(ret[i].Score), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
            cv::hconcat(loop_result, tmp_image, loop_result);
        }
    }

    if(frame_index - LOOP_SEARCH_GAP < 0)
    {
        return -1;
    }

     // a good match with its nerghbour
    if (ret.size() >= 1 &&ret[0].Score > 0.05)
    {
        for (unsigned int i = 1; i < ret.size(); i++)
        {
            //if (ret[i].Score > ret[0].Score * 0.3)
            if (ret[i].Score > 0.015)
            {          
                find_loop = true;
                int tmp_index = ret[i].Id;
                if (DEBUG_IMAGE && 0)
                {
                    auto it = image_pool.find(tmp_index);
                    cv::Mat tmp_image = (it->second).clone();
                    putText(tmp_image, "loop score:" + to_string(ret[i].Score), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,0,0),2);
                    cv::hconcat(loop_result, tmp_image, loop_result);
                }
            }

        }
    }   

    if(DEBUG_IMAGE)
    {   
        printf("show loop result\n");
        cv::imshow("loop result", loop_result);
        cv::waitKey(2);
    }

    if (find_loop && frame_index > 5)
    {
        int min_index = -1;
        for (unsigned int i = 0; i < ret.size(); i++)
        {
            if (min_index == -1 || (ret[i].Id < min_index && ret[i].Score > 0.015))
                min_index = ret[i].Id;
        }
        return min_index;
    }
    else
        return -1;
}

void LoopDetector::addKeyFrameIntoVoc(KeyFrame* keyframe)
{
    // cv::Mat compressed_image;
    // if (DEBUG_IMAGE)
    // {
    //     int feature_num = keyframe->keypoints.size();
    //     cv::resize(keyframe->image, compressed_image, cv::Size(376, 240));
    //     putText(compressed_image, "feature_num:" + to_string(feature_num), cv::Point2f(10, 10), CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
    //     image_pool[keyframe->index] = compressed_image;
    // }

    db.add(keyframe->brief_descriptors);
}
