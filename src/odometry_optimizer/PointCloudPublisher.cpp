#include "PointCloudPublisher.h"

#include <pcl_ros/transforms.h>
#include <utility>

PointCloudPublisher::PointCloudPublisher(string publishFrame, ros::Publisher &pub, int interval)
        : publishFrame(move(publishFrame)),
          pub(pub),
          interval(interval) {}

void PointCloudPublisher::republishInNewFrame(const sensor_msgs::PointCloud2 &msg) {
    counter++;
    if ((counter % interval) != 0) {
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
    if(!tfListener.waitForTransform("/aft_mapped_to_init_CORRECTED", "/camera_init", time, ros::Duration(3.0))) {
        cout << "Did not find transform, so not publishing point cloud." << endl;
        return;
    }
    tfListener.lookupTransform("/aft_mapped_to_init_CORRECTED", "/camera_init", time, transform);
    pcl_ros::transformPointCloud("/map", transform, msg, outMsg);

    // Set time and frame
    outMsg.header.stamp = time;
    outMsg.header.frame_id = "/velodyne_fused";

    // Publish
    pub.publish(outMsg);
}
