#include "visualizer/DepthMapUtil.h"

DepthCompletion::DepthCompletion(){}

DepthCompletion::~DepthCompletion(){}

void DepthCompletion::depthCompletionFast(cv::Mat &depth_map)
{
    cv::Mat kernel_mat;
    cv::Mat tmp_mat = depth_map.clone();
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

    cv::medianBlur(dilate_mat, tmp_mat,5);
    
    cv::Mat result;
    if(BLUR_TYPE == "bilateral")
    {
        cv::bilateralFilter(tmp_mat,result,5,1.5,2.0);
    }else
    {
        cv::GaussianBlur(tmp_mat,result, cv::Size(5,5), 0);
    }

    depth_map = result.clone();
}

void DepthCompletion::depthCompletionMultiScale(cv::Mat &depth_map)
{

}