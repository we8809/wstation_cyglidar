#include "Segmentation.h"

Segmentation::Segmentation(ros::NodeHandle* nodeHandle)
    : mVoxelLeafSize(0.0)
    , mRansacDistanceThreshold(0.0)
    , mPassThroughAxisXMin(0.0)
    , mPassThroughAxisXMax(0.0)
    , mPassThroughAxisYMin(0.0)
    , mPassThroughAxisYMax(0.0)
    , mPassThroughAxisZMin(0.0)
    , mPassThroughAxisZMax(0.0)
    , mSorMeanK(0)
    , mSorMultiThreshold(0.0)
{
    mNodeHandle = nodeHandle;

    /* voxel size param */
    mNodeHandle->getParam("/voxel_leaf_size", mVoxelLeafSize);

    /* ransac dist thres param */
    mNodeHandle->getParam("/ransac_distance_threshold", mRansacDistanceThreshold);

    /* passthrough filter param */
    mNodeHandle->getParam("/pass_through_x_axis_min", mPassThroughAxisXMin);
    mNodeHandle->getParam("/pass_through_x_axis_max", mPassThroughAxisXMax);
    mNodeHandle->getParam("/pass_through_y_axis_min", mPassThroughAxisYMin);
    mNodeHandle->getParam("/pass_through_y_axis_max", mPassThroughAxisYMax);
    mNodeHandle->getParam("/pass_through_z_axis_min", mPassThroughAxisZMin);
    mNodeHandle->getParam("/pass_through_z_axis_max", mPassThroughAxisZMax);

    /* statistic outlier filter param */
    mNodeHandle->getParam("/sor_mean_k", mSorMeanK);
    mNodeHandle->getParam("/sor_multi_threshold", mSorMultiThreshold);
}


void Segmentation::SetPassThroughFilter(const pcl::PointCloud<PointT>::Ptr& cloud)
{
    pcl::PassThrough<PointT> ptFilter;

    ptFilter.setInputCloud(cloud);
    ptFilter.setFilterFieldName("x");
    ptFilter.setFilterLimits(mPassThroughAxisXMin, mPassThroughAxisXMax);
    ptFilter.filter(*cloud);

    ptFilter.setInputCloud(cloud);
    ptFilter.setFilterFieldName("y");
    ptFilter.setFilterLimits(mPassThroughAxisYMin, mPassThroughAxisYMax);
    ptFilter.filter(*cloud);

    ptFilter.setInputCloud(cloud);
    ptFilter.setFilterFieldName("z");
    ptFilter.setFilterLimits(mPassThroughAxisZMin, mPassThroughAxisZMax);
    ptFilter.filter(*cloud);
}

void Segmentation::SetRansacPlanedFilter(const pcl::PointCloud<PointT>::Ptr& cloud)
{   
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ExtractIndices<PointT> extract;

    pcl::SACSegmentation<PointT> segmentation;

    // Optional
    segmentation.setOptimizeCoefficients(true);
    // Mandatory
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(mRansacDistanceThreshold);
    segmentation.setInputCloud(cloud);
    segmentation.segment(*inliers, *coefficients);

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud);
}

void Segmentation::SetStatisticalOutlierFilter(const pcl::PointCloud<PointT>::Ptr& cloud)
{
    pcl::StatisticalOutlierRemoval<PointT> sorFilter;
    sorFilter.setInputCloud(cloud);
    sorFilter.setMeanK(mSorMeanK);
    sorFilter.setStddevMulThresh(mSorMultiThreshold);
    sorFilter.filter(*cloud);
}
