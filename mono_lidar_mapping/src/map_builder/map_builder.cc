#include "map_builder/map_builder.h"

MapBuilder::MapBuilder(){}
MapBuilder::~MapBuilder(){}


void MapBuilder::depthMap(const Eigen::Matrix4d &transformation, const pcl::PointCloud<pcl::PointXYZ> &cloud, 
                        cv::Mat &frame,const double t)
{

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
    
    cv::Mat dilate_mat;
    cv::dilate(tmp_mat, dilate_mat,kernel_mat);

    cv::Mat hole_fill;
    cv::morphologyEx(dilate_mat,hole_fill,cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT,cv::Size(KERNEL_SIZE,KERNEL_SIZE)));

    cv::dilate(hole_fill,dilate_mat,cv::getStructuringElement(cv::MORPH_RECT,cv::Size(7,7)));

    //extend highest to the top of image
    cv::medianBlur(dilate_mat, tmp_mat,5);
    
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
