#include <ros/ros.h>

#include "CloudSubscriber.h"

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "wstation_cyglidar");
    ros::NodeHandle nodeHandle;
    ros::Rate loopRate(10);

    // CloudSubscriber cloudSubscriber(&nodeHandle, "scan_3D");
    CloudSubscriber cloudSubscriber(&nodeHandle, "/k4a/point_cloud/point_cloud_color_to_depth");
    cloudSubscriber.StartReveiveClouds();

    while (ros::ok()) 
    {
        ros::spinOnce();

        loopRate.sleep();
    }

    return 0;
}