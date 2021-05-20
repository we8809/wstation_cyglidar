#ifndef INCLUDE_CLOUD_SUBSCRIBER_H_
#define INCLUDE_CLOUD_SUBSCRIBER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>

#include <string>

/* pcl visualizer */
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "Segmentation.h"

class CloudSubscriber 
{
    using PointCloudMessage = sensor_msgs::PointCloud2;
    using PointT = pcl::PointXYZRGBA;

public:
    CloudSubscriber(ros::NodeHandle* nodeHandle, const std::string& cloudTopic);
    virtual ~CloudSubscriber();

    void StartReveiveClouds();

    void Callback(const sensor_msgs::PointCloud2::ConstPtr& cloudMsg);

private: 
    ros::NodeHandle* mNodeHandle;
    Segmentation* mSegmentation;

    message_filters::Subscriber<sensor_msgs::PointCloud2>* mCloudSubscriber;
    std::string mCloudTopic;

    int mMsgQueueSize;

    pcl::visualization::PCLVisualizer mViewer;
};

#endif /* INCLUDE_CLOUD_SUBSCRIBER_H_ */