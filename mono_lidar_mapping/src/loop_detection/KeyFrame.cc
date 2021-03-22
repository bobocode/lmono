#include "loop_detection/KeyFrame.h"

template <typename Derived>
static void reduceVector(vector<Derived> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

// create keyframe online
KeyFrame::KeyFrame(double _time_stamp, int _index, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, cv::Mat &_image,
		           vector<cv::Point3f> &_point_3d, vector<cv::Point2f> &_point_2d_uv, vector<cv::Point2f> &_point_2d_norm,
		           vector<int> &_point_id, int _sequence)
{
	time_stamp = _time_stamp;
	index = _index;
	vio_T_w_i = _vio_T_w_i;
	vio_R_w_i = _vio_R_w_i;
	T_w_i = vio_T_w_i;
	R_w_i = vio_R_w_i;
	origin_vio_T = vio_T_w_i;		
	origin_vio_R = vio_R_w_i;

	if(_image.channels() != 1)
	{
		cv::Mat tmp_img;
		cv::cvtColor(_image, tmp_img,cv::COLOR_BGR2GRAY,1);
		image = tmp_img.clone();

	}else
	{
		image = _image.clone();
	}
	
	cv::resize(image, thumbnail, cv::Size(80, 60));
	point_3d = _point_3d;
	point_2d_uv = _point_2d_uv;
	point_2d_norm = _point_2d_norm;
	point_id = _point_id;
	has_loop = false;
	loop_index = -1;
	has_fast_point = false;
	loop_info << 0, 0, 0, 0, 0, 0, 0, 0;
	sequence = _sequence;

	// printf("initial point 3d size %d\n",point_3d.size());
	// printf("initial point 2d size %d\n", point_2d_uv.size());
	// printf("initial point 2d norm size %d\n", point_2d_norm.size());
	// printf("initial point id size %d\n",point_id.size());

	computeWindowBRIEFPoint();
	computeBRIEFPoint();
	if(!DEBUG_IMAGE)
		image.release();
}

// load previous keyframe
KeyFrame::KeyFrame(double _time_stamp, int _index, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, Vector3d &_T_w_i, Matrix3d &_R_w_i,
					cv::Mat &_image, int _loop_index, Eigen::Matrix<double, 8, 1 > &_loop_info,
					vector<cv::KeyPoint> &_keypoints, vector<cv::KeyPoint> &_keypoints_norm, vector<BRIEF::bitset> &_brief_descriptors)
{
	time_stamp = _time_stamp;
	index = _index;
	//vio_T_w_i = _vio_T_w_i;
	//vio_R_w_i = _vio_R_w_i;
	vio_T_w_i = _T_w_i;
	vio_R_w_i = _R_w_i;
	T_w_i = _T_w_i;
	R_w_i = _R_w_i;
	if (DEBUG_IMAGE)
	{
		if(_image.channels() != 1)
		{
			cv::Mat tmp_img;
			cv::cvtColor(_image, tmp_img,cv::COLOR_BGR2GRAY,1);
			image = tmp_img.clone();

		}else
		{
			image = _image.clone();
		}
		cv::resize(image, thumbnail, cv::Size(80, 60));
	}
	if (_loop_index != -1)
		has_loop = true;
	else
		has_loop = false;
	loop_index = _loop_index;
	loop_info = _loop_info;
	has_fast_point = false;
	sequence = 0;
	brief_keypoints = _keypoints;
	brief_keypoints_norm = _keypoints_norm;
	brief_descriptors = _brief_descriptors;
}

// void KeyFrame::loadCam()
// {
// 	std::string file_name = CAM0;
// 	m_camera = 
// }
/*
void KeyFrame::computeORBPoint()
{
	std::vector<uchar> status;
	cv::Ptr<cv::ORB> orb_detector = cv::ORB::create(500,1.2f, 8,5);

	orb_detector->detect(image, orb_keypoints,MASK);

	for (int i = 0; i < (int)orb_keypoints.size(); i++)
	{
		Eigen::Vector3d tmp_p;
		m_camera->liftProjective(Eigen::Vector2d(orb_keypoints[i].pt.x, orb_keypoints[i].pt.y), tmp_p);
		cv::KeyPoint tmp_norm;
		tmp_norm.pt = cv::Point2f(tmp_p.x()/tmp_p.z(), tmp_p.y()/tmp_p.z());
		orb_keypoints_norm.push_back(tmp_norm);
	}

	orb_detector->compute(image,orb_keypoints,orb_descriptors);
}

void KeyFrame::computeWindowORBPoint()
{
	for(int i = 0; i < (int)point_2d_uv.size(); i++)
	{
	    cv::KeyPoint key;
	    key.pt = point_2d_uv[i];
	    orb_window_keypoints.push_back(key);
	}
	cv::Ptr<cv::ORB> orb_detector = cv::ORB::create();
	orb_detector->compute(image,orb_window_keypoints,window_orb_descriptors);
}
*/
void KeyFrame::computeWindowBRIEFPoint()
{
	BriefExtractor extractor(BRIEF_PATTERN_FILE.c_str());
	for(int i = 0; i < (int)point_2d_uv.size(); i++)
	{
	    cv::KeyPoint key;
	    key.pt = point_2d_uv[i];
	    brief_window_keypoints.push_back(key);
	}
	extractor(image, brief_window_keypoints, window_brief_descriptors);
}

void KeyFrame::computeBRIEFPoint()
{
	BriefExtractor extractor(BRIEF_PATTERN_FILE.c_str());
	const int fast_th = 20; // corner detector response threshold
	if(1)
		cv::FAST(image, brief_keypoints, fast_th, true);
	else
	{
		vector<cv::Point2f> tmp_pts;
		cv::goodFeaturesToTrack(image, tmp_pts, 500, 0.01, 10);
		for(int i = 0; i < (int)tmp_pts.size(); i++)
		{
		    cv::KeyPoint key;
		    key.pt = tmp_pts[i];
		    brief_keypoints.push_back(key);
		}
	}
	extractor(image, brief_keypoints, brief_descriptors);
	for (int i = 0; i < (int)brief_keypoints.size(); i++)
	{
		Eigen::Vector3d tmp_p;
		m_camera->liftProjective(Eigen::Vector2d(brief_keypoints[i].pt.x, brief_keypoints[i].pt.y), tmp_p);
		cv::KeyPoint tmp_norm;
		tmp_norm.pt = cv::Point2f(tmp_p.x()/tmp_p.z(), tmp_p.y()/tmp_p.z());
		brief_keypoints_norm.push_back(tmp_norm);
	}
}

void BriefExtractor::operator() (const cv::Mat &im, vector<cv::KeyPoint> &keys, vector<BRIEF::bitset> &descriptors) const
{
  m_brief.compute(im, keys, descriptors);
}

bool KeyFrame::searchInAera(const BRIEF::bitset window_descriptor,
                            const std::vector<BRIEF::bitset> &descriptors_old,
                            const std::vector<cv::KeyPoint> &keypoints_old,
                            const std::vector<cv::KeyPoint> &keypoints_old_norm,
                            cv::Point2f &best_match,
                            cv::Point2f &best_match_norm)
{
    cv::Point2f best_pt;
    int bestDist = 128;
    int bestIndex = -1;
    for(int i = 0; i < (int)descriptors_old.size(); i++)
    {

        int dis = HammingDis(window_descriptor, descriptors_old[i]);
        if(dis < bestDist)
        {
            bestDist = dis;
            bestIndex = i;
        }
    }
    //printf("best dist %d", bestDist);
    if (bestIndex != -1 && bestDist < 80)
    {
      best_match = keypoints_old[bestIndex].pt;
      best_match_norm = keypoints_old_norm[bestIndex].pt;
      return true;
    }
    else
      return false;
}

void KeyFrame::searchByBRIEFDes(std::vector<cv::Point2f> &matched_2d_old,
								std::vector<cv::Point2f> &matched_2d_old_norm,
                                std::vector<uchar> &status,
                                const std::vector<BRIEF::bitset> &descriptors_old,
                                const std::vector<cv::KeyPoint> &keypoints_old,
                                const std::vector<cv::KeyPoint> &keypoints_old_norm)
{
    for(int i = 0; i < (int)window_brief_descriptors.size(); i++)
    {
        cv::Point2f pt(0.f, 0.f);
        cv::Point2f pt_norm(0.f, 0.f);
        if (searchInAera(window_brief_descriptors[i], descriptors_old, keypoints_old, keypoints_old_norm, pt, pt_norm))
          status.push_back(1);
        else
          status.push_back(0);
        matched_2d_old.push_back(pt);
        matched_2d_old_norm.push_back(pt_norm);
    }

}


void KeyFrame::FundmantalMatrixRANSAC(const std::vector<cv::Point2f> &matched_2d_cur_norm,
                                      const std::vector<cv::Point2f> &matched_2d_old_norm,
                                      vector<uchar> &status)
{
	int n = (int)matched_2d_cur_norm.size();
	for (int i = 0; i < n; i++)
		status.push_back(0);
    if (n >= 8)
    {
        vector<cv::Point2f> tmp_cur(n), tmp_old(n);
        for (int i = 0; i < (int)matched_2d_cur_norm.size(); i++)
        {
            double FOCAL_LENGTH = 460.0;
            double tmp_x, tmp_y;
            tmp_x = FOCAL_LENGTH * matched_2d_cur_norm[i].x + COL / 2.0;
            tmp_y = FOCAL_LENGTH * matched_2d_cur_norm[i].y + ROW / 2.0;
            tmp_cur[i] = cv::Point2f(tmp_x, tmp_y);

            tmp_x = FOCAL_LENGTH * matched_2d_old_norm[i].x + COL / 2.0;
            tmp_y = FOCAL_LENGTH * matched_2d_old_norm[i].y + ROW / 2.0;
            tmp_old[i] = cv::Point2f(tmp_x, tmp_y);
        }
        cv::findFundamentalMat(tmp_cur, tmp_old, cv::FM_RANSAC, 3.0, 0.9, status);
    }
}

void KeyFrame::PnPRANSAC(const vector<cv::Point2f> &matched_2d_old_norm,
                         const std::vector<cv::Point3f> &matched_3d,
                         std::vector<uchar> &status,
                         Eigen::Vector3d &PnP_T_old, Eigen::Matrix3d &PnP_R_old)
{
	// for (int i = 0; i < matched_3d.size(); i++)
	// 	printf("3d x: %f, y: %f, z: %f\n",matched_3d[i].x, matched_3d[i].y, matched_3d[i].z );
	// printf("match size %d \n", matched_3d.size());
    cv::Mat r, rvec, t, D, tmp_r;
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);
    Matrix3d R_inital;
    Vector3d P_inital;
    Matrix3d R_w_c = origin_vio_R * qlc;
    Vector3d T_w_c = origin_vio_T + origin_vio_R * tlc;

    R_inital = R_w_c.inverse();
    P_inital = -(R_inital * T_w_c);

    cv::eigen2cv(R_inital, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_inital, t);

    cv::Mat inliers;
    TicToc t_pnp_ransac;

    if (CV_MAJOR_VERSION < 3)
        solvePnPRansac(matched_3d, matched_2d_old_norm, K, D, rvec, t, true, 100, 10.0 / 460.0, 100, inliers);
    else
    {
        if (CV_MINOR_VERSION < 2)
            solvePnPRansac(matched_3d, matched_2d_old_norm, K, D, rvec, t, true, 100, sqrt(10.0 / 460.0), 0.99, inliers);
        else
            solvePnPRansac(matched_3d, matched_2d_old_norm, K, D, rvec, t, true, 100, 10.0 / 460.0, 0.99, inliers);

    }

    for (int i = 0; i < (int)matched_2d_old_norm.size(); i++)
        status.push_back(0);

    for( int i = 0; i < inliers.rows; i++)
    {
        int n = inliers.at<int>(i);
        status[n] = 1;
    }

    cv::Rodrigues(rvec, r);
    Matrix3d R_pnp, R_w_c_old;
    cv::cv2eigen(r, R_pnp);
    R_w_c_old = R_pnp.transpose();
    Vector3d T_pnp, T_w_c_old;
    cv::cv2eigen(t, T_pnp);
    T_w_c_old = R_w_c_old * (-T_pnp);

    PnP_R_old = R_w_c_old * qlc.transpose();
    PnP_T_old = T_w_c_old - PnP_R_old * tlc;
}


bool KeyFrame::findConnection(KeyFrame* old_kf)
{
	TicToc tmp_t;
	printf("find Connection\n");
	vector<cv::Point2f> matched_2d_cur, matched_2d_old;
	vector<cv::Point2f> matched_2d_cur_norm, matched_2d_old_norm;
	vector<cv::Point3f> matched_3d;
	vector<int> matched_id;
	vector<uchar> status;

	matched_3d = point_3d;
	matched_2d_cur = point_2d_uv;
	matched_2d_cur_norm = point_2d_norm;
	matched_id = point_id;

	// printf("initial matched 3d size %d\n",matched_3d.size());
	// printf("initial match 2d size %d\n", matched_2d_cur.size());
	// printf("initial match 2d norm size %d\n", matched_2d_cur_norm.size());
	// printf("initial matched id size %d\n",matched_id.size());
	TicToc t_match;
	
	// if (DEBUG_IMAGE)    
	// {
	// 	cv::Mat gray_img, loop_match_img;
	// 	//printf("finding old image\n");
	// 	cv::Mat old_img = old_kf->image;

	// 	//printf("finding old image\n");
	// 	cv::hconcat(image, old_img, gray_img);
	// 	cvtColor(gray_img, loop_match_img, CV_GRAY2RGB);
	// 	//printf("concat image\n");
	// 	for(int i = 0; i< (int)point_2d_uv.size(); i++)
	// 	{
	// 		cv::Point2f cur_pt = point_2d_uv[i];
	// 		cv::circle(loop_match_img, cur_pt, 5, cv::Scalar(0, 255, 0));
	// 	}
	// 	for(int i = 0; i< (int)old_kf->brief_keypoints.size(); i++)
	// 	{
	// 		cv::Point2f old_pt = old_kf->brief_keypoints[i].pt;
	// 		old_pt.x += COL;
	// 		cv::circle(loop_match_img, old_pt, 5, cv::Scalar(0, 255, 0));
	// 	}
	// 	ostringstream path;
	// 	path << "/home/bo/raw_data/loop_image/"
	// 			<< index << "-"
	// 			<< old_kf->index << "-" << "0raw_point.jpg";
	// 	printf("loop match img\n");
	// 	cv::imwrite( path.str().c_str(), loop_match_img);
	// }
	
	//printf("search by des\n");
	searchByBRIEFDes(matched_2d_old, matched_2d_old_norm, status, old_kf->brief_descriptors, old_kf->brief_keypoints, old_kf->brief_keypoints_norm);
	reduceVector(matched_2d_cur, status);
	reduceVector(matched_2d_old, status);
	reduceVector(matched_2d_cur_norm, status);
	reduceVector(matched_2d_old_norm, status);
	reduceVector(matched_3d, status);
	reduceVector(matched_id, status);
	//printf("search by des finish\n");

	// printf("des matched 3d size %d\n",matched_3d.size());
	// printf("des match 2d size %d\n", matched_2d_cur.size());
	// printf("des match 2d norm size %d\n", matched_2d_cur_norm.size());

	// if ((int)matched_2d_cur.size() > MIN_BRIEF_LOOP_NUM)
	// {
	// 	if (DEBUG_IMAGE)
	// 	{
	// 		int gap = 10;
	// 		cv::Mat gap_image(ROW, gap, CV_8UC1, cv::Scalar(255, 255, 255));
	// 		cv::Mat gray_img, loop_match_img;
	// 		cv::Mat old_img = old_kf->image;
	// 		cv::hconcat(image, gap_image, gap_image);
	// 		cv::hconcat(gap_image, old_img, gray_img);
	// 		cvtColor(gray_img, loop_match_img, CV_GRAY2RGB);
	// 		for(int i = 0; i< (int)matched_2d_cur.size(); i++)
	// 		{
	// 			cv::Point2f cur_pt = matched_2d_cur[i];
	// 			cv::circle(loop_match_img, cur_pt, 5, cv::Scalar(0, 255, 0));
	// 		}
	// 		for(int i = 0; i< (int)matched_2d_old.size(); i++)
	// 		{
	// 			cv::Point2f old_pt = matched_2d_old[i];
	// 			old_pt.x += (COL + gap);
	// 			cv::circle(loop_match_img, old_pt, 5, cv::Scalar(0, 255, 0));
	// 		}
	// 		for (int i = 0; i< (int)matched_2d_cur.size(); i++)
	// 		{
	// 			cv::Point2f old_pt = matched_2d_old[i];
	// 			old_pt.x +=  (COL + gap);
	// 			cv::line(loop_match_img, matched_2d_cur[i], old_pt, cv::Scalar(0, 255, 0), 1, 8, 0);
	// 		}

	// 		ostringstream path, path1, path2;
	// 		path <<  "/home/bo/raw_data/loop_image/"
	// 				<< index << "-"
	// 				<< old_kf->index << "-" << "1descriptor_match.jpg";
	// 		cv::imwrite( path.str().c_str(), loop_match_img);
	// 	}
	// }
	status.clear();
	/*
	FundmantalMatrixRANSAC(matched_2d_cur_norm, matched_2d_old_norm, status);
	reduceVector(matched_2d_cur, status);
	reduceVector(matched_2d_old, status);
	reduceVector(matched_2d_cur_norm, status);
	reduceVector(matched_2d_old_norm, status);
	reduceVector(matched_3d, status);
	reduceVector(matched_id, status);
	*/
	
	// if (DEBUG_IMAGE)
	// {
	// 	int gap = 10;
	// 	cv::Mat gap_image(ROW, gap, CV_8UC1, cv::Scalar(255, 255, 255));
	// 	cv::Mat gray_img, loop_match_img;
	// 	cv::Mat old_img = old_kf->image;
	// 	cv::hconcat(image, gap_image, gap_image);
	// 	cv::hconcat(gap_image, old_img, gray_img);
	// 	cvtColor(gray_img, loop_match_img, CV_GRAY2RGB);
	// 	for(int i = 0; i< (int)matched_2d_cur.size(); i++)
	// 	{
	// 		cv::Point2f cur_pt = matched_2d_cur[i];
	// 		cv::circle(loop_match_img, cur_pt, 5, cv::Scalar(0, 255, 0));
	// 	}
	// 	for(int i = 0; i< (int)matched_2d_old.size(); i++)
	// 	{
	// 		cv::Point2f old_pt = matched_2d_old[i];
	// 		old_pt.x += (COL + gap);
	// 		cv::circle(loop_match_img, old_pt, 5, cv::Scalar(0, 255, 0));
	// 	}
	// 	for (int i = 0; i< (int)matched_2d_cur.size(); i++)
	// 	{
	// 		cv::Point2f old_pt = matched_2d_old[i];
	// 		old_pt.x +=  (COL + gap) ;
	// 		cv::line(loop_match_img, matched_2d_cur[i], old_pt, cv::Scalar(0, 255, 0), 1, 8, 0);
	// 	}

	// 	ostringstream path;
	// 	path <<  "/home/bo/raw_data/loop_image/"
	// 			<< index << "-"
	// 			<< old_kf->index << "-" << "2fundamental_match.jpg";
	// 	cv::imwrite( path.str().c_str(), loop_match_img);
	// }
	
	Eigen::Vector3d PnP_T_old;
	Eigen::Matrix3d PnP_R_old;
	Eigen::Vector3d relative_t;
	Quaterniond relative_q;
	Eigen::Vector3d relative_euler;
	double relative_yaw;
	if ((int)matched_2d_cur.size() > MIN_BRIEF_LOOP_NUM)
	{
		status.clear();
	    PnPRANSAC(matched_2d_old_norm, matched_3d, status, PnP_T_old, PnP_R_old);
	    reduceVector(matched_2d_cur, status);
	    reduceVector(matched_2d_old, status);
	    reduceVector(matched_2d_cur_norm, status);
	    reduceVector(matched_2d_old_norm, status);
	    reduceVector(matched_3d, status);
	    reduceVector(matched_id, status);
	
	}

	if ((int)matched_2d_cur.size() > MIN_PNP_LOOP_NUM)
	{
	    relative_t = PnP_R_old.transpose() * (origin_vio_T - PnP_T_old);
	    relative_q = PnP_R_old.transpose() * origin_vio_R;
	    relative_yaw = mathutils::normalizeAngle(mathutils::R2ypr(origin_vio_R).x() - mathutils::R2ypr(PnP_R_old).x());
		relative_euler = mathutils::R2ypr(origin_vio_R) - mathutils::R2ypr(PnP_R_old);
	    printf("PNP relative\n");
	    std::cout << "pnp relative_t " << relative_t.transpose() << std::endl;
	    std::cout << "pnp relative_yaw " << relative_yaw << std::endl;
		std::cout << "pnp relative euler " << relative_euler.transpose() << std::endl;
		std::cout << "pnp relative euler norm " << relative_euler.norm() << std::endl;
		std::cout << "pnp relative_t norm " << relative_t.norm() << std::endl;

		// if(distributionValidation(matched_2d_cur,matched_2d_old))
		// {
		// 	return true;
		// }

	    if (abs(relative_euler.norm()) < 30.0 && relative_t.norm() < 20.0)
	    {
			point_loop_2d_norm = matched_2d_cur_norm;
			point_old_2d_norm = matched_2d_old_norm;
			point_loop_id = matched_id;

			if (DEBUG_IMAGE)
			{
				int gap = 10;
				cv::Mat gap_image(ROW, gap, CV_8UC1, cv::Scalar(255, 255, 255));
				cv::Mat gray_img, loop_match_img;
				cv::Mat old_img = old_kf->image;
				cv::hconcat(image, gap_image, gap_image);
				cv::hconcat(gap_image, old_img, gray_img);
				cvtColor(gray_img, loop_match_img, CV_GRAY2RGB);
				for(int i = 0; i< (int)matched_2d_cur.size(); i++)
				{
					cv::Point2f cur_pt = matched_2d_cur[i];
					cv::circle(loop_match_img, cur_pt, 5, cv::Scalar(0, 255, 0));
				}
				for(int i = 0; i< (int)matched_2d_old.size(); i++)
				{
					cv::Point2f old_pt = matched_2d_old[i];
					old_pt.x += (COL + gap);
					cv::circle(loop_match_img, old_pt, 5, cv::Scalar(0, 255, 0));
				}
				for (int i = 0; i< (int)matched_2d_cur.size(); i++)
				{
					cv::Point2f old_pt = matched_2d_old[i];
					old_pt.x += (COL + gap) ;
					cv::line(loop_match_img, matched_2d_cur[i], old_pt, cv::Scalar(0, 255, 0), 2, 8, 0);
				}
				cv::Mat notationR(50, COL + gap + COL, CV_8UC3, cv::Scalar(255, 255, 255));
				putText(notationR, "relative t: " + to_string(relative_t(0)) + " " + to_string(relative_t(1)) + " " + to_string(relative_t(2)), cv::Point2f(20, 30), CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255), 3);
				cv::Mat notation(50, COL + gap + COL, CV_8UC3, cv::Scalar(255, 255, 255));
				putText(notation, "current frame: " + to_string(time_stamp) + "  sequence: " + to_string(sequence) + " matched size: " + to_string(matched_2d_cur.size()), cv::Point2f(20, 30), CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255), 3);

				putText(notation, "previous frame: " + to_string(old_kf->time_stamp) + "  sequence: " + to_string(old_kf->sequence), cv::Point2f(20 + COL + gap, 30), CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255), 3);
				cv::vconcat(notation, loop_match_img, loop_match_img);
				cv::vconcat(loop_match_img, notationR, loop_match_img);
				
				ostringstream path;
				
				path <<  "/home/bo/raw_data/loop_image/"
						<< index << "-"
						<< old_kf->index << "-" << "3pnp_match.jpg";
				cv::imwrite( path.str().c_str(), loop_match_img);
			}
	    	has_loop = true;
	    	loop_index = old_kf->index;
	    	loop_info << relative_t.x(), relative_t.y(), relative_t.z(),
	    	             relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
	    	             relative_yaw;

			printf(RED"publish feature points %f\n" WHT,time_stamp);

			sensor_msgs::PointCloud matched_points_msg;
			matched_points_msg.header.stamp = ros::Time(time_stamp);
			for(size_t i =0; i < matched_2d_old_norm.size(); i++)
			{
				geometry_msgs::Point32 p;
				p.x = matched_2d_old_norm[i].x;
				p.y = matched_2d_old_norm[i].y;
				p.z = (float)matched_id[i];

				printf(RED "point id %d : (%f, %f)\n" WHT, (int)p.z, p.x, p.y);

				matched_points_msg.points.push_back(p);
			}

			Eigen::Vector3d old_T = old_kf->T_w_i;
			Eigen::Matrix3d old_R = old_kf->R_w_i;
			Eigen::Quaterniond old_Q(old_R);

			Eigen::Vector3d correct_T = old_R * relative_t + old_T;
            Eigen::Matrix3d correct_R = old_R * relative_q.toRotationMatrix();
			Eigen::Quaterniond correct_Q(correct_R);

			sensor_msgs::ChannelFloat32 t_q_index;
			t_q_index.values.push_back(old_T.x());
			t_q_index.values.push_back(old_T.y());
			t_q_index.values.push_back(old_T.z());
			t_q_index.values.push_back(old_Q.w());
			t_q_index.values.push_back(old_Q.x());
			t_q_index.values.push_back(old_Q.y());
			t_q_index.values.push_back(old_Q.z());
			t_q_index.values.push_back(correct_T.x());
			t_q_index.values.push_back(correct_T.y());
			t_q_index.values.push_back(correct_T.z());
			t_q_index.values.push_back(correct_Q.w());
			t_q_index.values.push_back(correct_Q.x());
			t_q_index.values.push_back(correct_Q.y());
			t_q_index.values.push_back(correct_Q.z());
			t_q_index.values.push_back(index);
			matched_points_msg.channels.push_back(t_q_index);

			pub_matched_points_.publish(matched_points_msg);
			
	        return true;
	    }
	}
	printf("loop final use num %d %lf--------------- \n", (int)matched_2d_cur.size(), t_match.Toc());
	return false;
}

bool KeyFrame::distributionValidation(const vector<cv::Point2f>& new_point_2d_uv, const vector<cv::Point2f>& old_point_2d_uv)
{
	if (new_point_2d_uv.empty())
        return false;

    pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ> ());
    for (size_t i = 0; i < new_point_2d_uv.size(); ++i)
    {
        pcl::PointXYZ p;
        p.x = new_point_2d_uv[i].x;
        p.y = new_point_2d_uv[i].y;
        p.z = 0;
        new_cloud->push_back(p);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr old_cloud(new pcl::PointCloud<pcl::PointXYZ> ());
    for (size_t i = 0; i < old_point_2d_uv.size(); ++i)
    {
        pcl::PointXYZ p;
        p.x = old_point_2d_uv[i].x;
        p.y = old_point_2d_uv[i].y;
        p.z = 0;
        old_cloud->push_back(p);
    }

	Eigen::Vector4f new_xyz_centroid;
    Eigen::Matrix3f new_covariance_matrix;
    pcl::compute3DCentroid(*new_cloud, new_xyz_centroid);
    pcl::computeCovarianceMatrix(*new_cloud, new_xyz_centroid, new_covariance_matrix); 

    Eigen::Vector4f old_xyz_centroid;
    Eigen::Matrix3f old_covariance_matrix;
    pcl::compute3DCentroid(*old_cloud, old_xyz_centroid);
    pcl::computeCovarianceMatrix(*old_cloud, old_xyz_centroid, old_covariance_matrix);

    float new_cov_x = sqrt(new_covariance_matrix(0,0));
    float new_cov_y = sqrt(new_covariance_matrix(1,1));
    float old_cov_x = sqrt(old_covariance_matrix(0,0));
    float old_cov_y = sqrt(old_covariance_matrix(1,1));
    float cov_x_diff = abs(new_cov_x - old_cov_x);
    float cov_y_diff = abs(new_cov_y - old_cov_y);

	printf("cov_x_diff %f, cov_y_diff %f, new_cov_x %f, new_cov_y %f, old_cov_x %f, old_cov_y %f\n",
			cov_x_diff, cov_y_diff, new_cov_x, new_cov_y, old_cov_x, old_cov_y);

    if (cov_x_diff > 3 * std::min(new_cov_x, old_cov_x) || cov_y_diff > 0.75 * std::min(new_cov_y, old_cov_y))
    {
        return false;
    }

    return true;
}

int KeyFrame::HammingDis(const BRIEF::bitset &a, const BRIEF::bitset &b)
{
    BRIEF::bitset xor_of_bitset = a ^ b;
    int dis = xor_of_bitset.count();
    return dis;
}

void KeyFrame::getVioPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i)
{
    _T_w_i = vio_T_w_i;
    _R_w_i = vio_R_w_i;
}

void KeyFrame::getPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i)
{
    _T_w_i = T_w_i;
    _R_w_i = R_w_i;
}

void KeyFrame::updatePose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i)
{
    T_w_i = _T_w_i;
    R_w_i = _R_w_i;
}

void KeyFrame::updateVioPose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i)
{
	vio_T_w_i = _T_w_i;
	vio_R_w_i = _R_w_i;
	T_w_i = vio_T_w_i;
	R_w_i = vio_R_w_i;
}

Eigen::Vector3d KeyFrame::getLoopRelativeT()
{
    return Eigen::Vector3d(loop_info(0), loop_info(1), loop_info(2));
}

Eigen::Quaterniond KeyFrame::getLoopRelativeQ()
{
    return Eigen::Quaterniond(loop_info(3), loop_info(4), loop_info(5), loop_info(6));
}

double KeyFrame::getLoopRelativeYaw()
{
    return loop_info(7);
}

void KeyFrame::updateLoop(Eigen::Matrix<double, 8, 1 > &_loop_info)
{
	if (abs(_loop_info(7)) < 30.0 && Vector3d(_loop_info(0), _loop_info(1), _loop_info(2)).norm() < 20.0)
	{
		//printf("update loop info\n");
		loop_info = _loop_info;
	}
}

BriefExtractor::BriefExtractor(const std::string &pattern_file)
{
  // The DVision::BRIEF extractor computes a random pattern by default when
  // the object is created.
  // We load the pattern that we used to build the vocabulary, to make
  // the descriptors compatible with the predefined vocabulary

  // loads the pattern
  cv::FileStorage fs(pattern_file.c_str(), cv::FileStorage::READ);
  if(!fs.isOpened()) throw string("Could not open file ") + pattern_file;

  vector<int> x1, y1, x2, y2;
  fs["x1"] >> x1;
  fs["x2"] >> x2;
  fs["y1"] >> y1;
  fs["y2"] >> y2;

  m_brief.importPairs(x1, y1, x2, y2);
}


