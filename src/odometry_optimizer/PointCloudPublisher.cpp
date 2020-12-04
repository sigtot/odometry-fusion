#include "PointCloudPublisher.h"

#include <pcl_ros/transforms.h>

#include <utility>

PointCloudPublisher::PointCloudPublisher(ros::Publisher &pub, PointCloudPublisherParams params)
        : params(std::move(params)),
          pub(pub) {}

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
    pub.publish(outMsg);
}
