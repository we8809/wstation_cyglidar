#include <algorithm>

#include "CloudSubscriber.h"

CloudSubscriber::CloudSubscriber(ros::NodeHandle* nodeHandle, const std::string& cloudTopic) 
    : mMsgQueueSize(100)
{
    mNodeHandle = nodeHandle;
    mCloudTopic = cloudTopic;
    
    mCloudSubscriber = nullptr;

    mSegmentation = new Segmentation(mNodeHandle);
}

CloudSubscriber::~CloudSubscriber()
{
    delete mCloudSubscriber;
}

void CloudSubscriber::StartReveiveClouds()
{
    if (mCloudSubscriber != nullptr) 
    {   
        return;
    }

    mCloudSubscriber = new message_filters::Subscriber<sensor_msgs::PointCloud2>(*mNodeHandle, mCloudTopic, mMsgQueueSize);
    mCloudSubscriber->registerCallback(&CloudSubscriber::Callback, this);
}

void CloudSubscriber::Callback(const sensor_msgs::PointCloud2::ConstPtr& cloudMsg)
{
    /* View */
    mViewer.removeAllPointClouds();
    mViewer.removeAllShapes();
    mViewer.removeAllCoordinateSystems();


    pcl::PCLPointCloud2* tempCloud = new pcl::PCLPointCloud2();
    pcl::PCLPointCloud2ConstPtr tempCloudPtr(tempCloud);

    pcl::PointCloud<PointT> * cloud = new pcl::PointCloud<PointT>();
    pcl::PointCloud<PointT>::Ptr cloudPtr(cloud);

    pcl_conversions::toPCL(*cloudMsg, *tempCloud);
    pcl::fromPCLPointCloud2(*tempCloudPtr, *cloudPtr);

    // mSegmentation->SetPassThroughFilter(cloudPtr);
    // mSegmentation->SetRansacPlanedFilter(cloudPtr);

    pcl::visualization::PointCloudColorHandlerCustom<PointT> tc_handler(cloudPtr, 0, 255, 0); //Point cloud related to the origin
    mViewer.addPointCloud(cloudPtr, tc_handler, "cloud");
    mViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

    mViewer.setBackgroundColor(0,0,0);
    mViewer.addCoordinateSystem(1.0);
    mViewer.resetCamera();
    
    mViewer.spinOnce();
}
