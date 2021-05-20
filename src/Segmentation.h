#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

/* pcl filters */
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>

/* pcl ransac */
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>

class Segmentation 
{
    using PointT = pcl::PointXYZRGBA;

public:
    Segmentation(ros::NodeHandle* nodeHandle);
    virtual ~Segmentation() = default;

    void SetPassThroughFilter(const pcl::PointCloud<PointT>::Ptr& cloud);
    void SetRansacPlanedFilter(const pcl::PointCloud<PointT>::Ptr& cloud);
    void SetStatisticalOutlierFilter(const pcl::PointCloud<PointT>::Ptr& cloud);

private:
    ros::NodeHandle* mNodeHandle;

    double mVoxelLeafSize;
    double mRansacDistanceThreshold;

    double mPassThroughAxisXMin;
    double mPassThroughAxisXMax;
    double mPassThroughAxisYMin;
    double mPassThroughAxisYMax;
    double mPassThroughAxisZMin;
    double mPassThroughAxisZMax;

    int mSorMeanK;
    double mSorMultiThreshold;
};

#endif /* SEGMENTATION_H */