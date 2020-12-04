#ifndef ODOMETRY_OPTIMIZER_POINTCLOUDPUBLISHER_H
#define ODOMETRY_OPTIMIZER_POINTCLOUDPUBLISHER_H

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <mutex>
#include <string>
#include <vector>
#include <thread>
#include <ros/ros.h>
#include <tf/transform_listener.h>

using namespace std;


struct PointCloudPublisherParams {
    int interval;

    double finalPublishTime;

    string lidarFrameId; /// The point cloud will be transformed from this frame to lidarInitFrameId align it with origin
    string lidarInitFrameId; /// The point cloud will be transformed to this frame from lidarFrameId to align it with origin

    string liveFrameId; /// The frame in which the point cloud will be published during live execution
    string finalFrameId; /// The frame in which the final point cloud will be published when the complete trajectory is done
};

class PointCloudPublisher {
private:
    ros::Publisher &livePub;
    ros::Publisher &finalPub;
    tf::TransformListener tfListener;
    int counter = 0;
    const PointCloudPublisherParams params;
    vector<sensor_msgs::PointCloud2> pointClouds;

    nav_msgs::Path newestPath;
    mutex newestPathMutex;
    bool haveNewestPath = false;

    thread finalPathPublishThread;

    void waitForInactivityAndPublishFinalPointClouds();

    void publishFinalPointClouds();

public:
    PointCloudPublisher(ros::Publisher &livePub, ros::Publisher &finalPub,
                        PointCloudPublisherParams params);

    virtual ~PointCloudPublisher();

    void storeAndRepublishInNewFrame(const sensor_msgs::PointCloud2 &msg);

    void recvNewPath(const nav_msgs::Path &msg);
};


#endif //ODOMETRY_OPTIMIZER_POINTCLOUDPUBLISHER_H
