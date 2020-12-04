#include "PointCloudPublisher.h"

#include <pcl_ros/transforms.h>

#include <utility>

#include <cmath>

PointCloudPublisher::PointCloudPublisher(ros::Publisher &livePub, ros::Publisher &finalPub,
                                         PointCloudPublisherParams params)
        : params(std::move(params)),
          livePub(livePub),
          finalPub(finalPub) {
    finalPathPublishThread = thread(&PointCloudPublisher::waitForInactivityAndPublishFinalPointClouds, this);
}

void PointCloudPublisher::storeAndRepublishInNewFrame(const sensor_msgs::PointCloud2 &msg) {
    counter++;
    if ((counter % params.interval) != 0) {
        return;
    }
    // Create copy msg
    sensor_msgs::PointCloud2 outMsg;
    outMsg.data = msg.data;
    outMsg.point_step = msg.point_step;
    outMsg.row_step = msg.row_step;
    outMsg.fields = msg.fields;
    outMsg.width = msg.width;
    outMsg.height = msg.height;
    outMsg.is_bigendian = msg.is_bigendian;
    outMsg.is_dense = msg.is_dense;

    // Get and apply transform
    tf::StampedTransform transform;
    auto time = msg.header.stamp;
    if (!tfListener.waitForTransform(params.lidarFrameId, params.lidarInitFrameId, time, ros::Duration(3.0))) {
        cout << "Did not find transform, so not publishing point cloud." << endl;
        return;
    }
    tfListener.lookupTransform(params.lidarFrameId, params.lidarInitFrameId, time, transform);
    pcl_ros::transformPointCloud("/map", transform, msg,
                                 outMsg); // Actually don't know what that "/map" target frame means, but whatever. This works.

    // Set time and frame
    outMsg.header.stamp = time;
    outMsg.header.frame_id = params.liveFrameId;

    // Publish
    livePub.publish(outMsg);

    // For the final point clouds, we set the frame id to the (global) finalFrameId instead of the on-robot frame used for live updates
    outMsg.header.frame_id = params.finalFrameId;
    pointClouds.push_back(outMsg);
}

void PointCloudPublisher::publishFinalPointClouds() {
    for (auto &cloud : pointClouds) {
        auto closestPose = *newestPath.poses.begin();
        double closestTimeDiff = 99999;
        for (const auto &pose : newestPath.poses) {
            double timeDiff = abs(pose.header.stamp.toSec() - cloud.header.stamp.toSec());
            if (timeDiff < closestTimeDiff) {
                closestPose = pose;
                closestTimeDiff = timeDiff;
            }
        }
        tf::Transform transform;
        tf::poseMsgToTF(closestPose.pose, transform);
        auto transformedCloud = cloud;
        pcl_ros::transformPointCloud("/dunno", transform, cloud, transformedCloud);

        transformedCloud.header.frame_id = params.finalFrameId;
        finalPub.publish(transformedCloud);
        this_thread::sleep_for(chrono::milliseconds(40));  // Add small delay between msgs for cool effect (and to not overload the publisher)
    }
}

void PointCloudPublisher::recvNewPath(const nav_msgs::Path &msg) {
    lock_guard<mutex> lock(newestPathMutex);
    newestPath = msg;
    haveNewestPath = true;
}

void PointCloudPublisher::waitForInactivityAndPublishFinalPointClouds() {
    while (!ros::isShuttingDown()) {
        {
            lock_guard<mutex> lock(newestPathMutex);
            if (haveNewestPath && newestPath.header.stamp.toSec() > params.finalPublishTime) {
                cout << "Publishing final point clouds" << endl;
                publishFinalPointClouds();
                return;
            }
        }
        this_thread::sleep_for(chrono::milliseconds(200));  //Sleep for 200ms ~ 5Hz polling
    }
}

PointCloudPublisher::~PointCloudPublisher() {
    finalPathPublishThread.join();
}
