#ifndef ODOMETRY_OPTIMIZER_POINTCLOUDPUBLISHER_H
#define ODOMETRY_OPTIMIZER_POINTCLOUDPUBLISHER_H

#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <ros/ros.h>
#include <tf/transform_listener.h>

using namespace std;


class PointCloudPublisher {
private:
    const string publishFrame;
    ros::Publisher &pub;
    tf::TransformListener tfListener;
    const int interval;
    int counter = 0;

public:
    PointCloudPublisher(string publishFrame, ros::Publisher &pub, int interval);
    void republishInNewFrame(const sensor_msgs::PointCloud2 &msg);

public:

};


#endif //ODOMETRY_OPTIMIZER_POINTCLOUDPUBLISHER_H
