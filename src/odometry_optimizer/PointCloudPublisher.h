#ifndef ODOMETRY_OPTIMIZER_POINTCLOUDPUBLISHER_H
#define ODOMETRY_OPTIMIZER_POINTCLOUDPUBLISHER_H

#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <ros/ros.h>
#include <tf/transform_listener.h>

using namespace std;


struct PointCloudPublisherParams {
    int interval;

    string lidarFrameId; /// The point cloud will be transformed from this frame to lidarInitFrameId align it with origin
    string lidarInitFrameId; /// The point cloud will be transformed to this frame from lidarFrameId to align it with origin

    string liveFrameId; /// The frame in which the point cloud will be published during live execution
    string finalFrameId; /// The frame in which the final point cloud will be published when the complete trajectory is done
};

class PointCloudPublisher {
private:
    ros::Publisher &pub;
    tf::TransformListener tfListener;
    int counter = 0;
    const PointCloudPublisherParams params;

public:
    PointCloudPublisher(ros::Publisher &pub, PointCloudPublisherParams params);

    void storeAndRepublishInNewFrame(const sensor_msgs::PointCloud2 &msg);

public:

};


#endif //ODOMETRY_OPTIMIZER_POINTCLOUDPUBLISHER_H
