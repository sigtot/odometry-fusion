#include "utils.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>

geometry_msgs::Pose toPoseMsg(const Pose3 &pose) {
    geometry_msgs::Point pointMsg;
    pointMsg.x = pose.x();
    pointMsg.y = pose.y();
    pointMsg.z = pose.z();

    geometry_msgs::Quaternion quatMsg;
    quatMsg.w = pose.rotation().quaternion().w();
    quatMsg.x = pose.rotation().quaternion().x();
    quatMsg.y = pose.rotation().quaternion().y();
    quatMsg.z = pose.rotation().quaternion().z();

    geometry_msgs::Pose poseMsg;
    poseMsg.position = pointMsg;
    poseMsg.orientation = quatMsg;
    return poseMsg;
}

Pose3 toPose3(const geometry_msgs::Pose &poseMsg) {
    auto pointMsg = poseMsg.position;
    auto point = Point3(pointMsg.x, pointMsg.y, pointMsg.z);
    auto quatMsg = poseMsg.orientation;
    auto rot = Rot3::Quaternion(quatMsg.w, quatMsg.x, quatMsg.y, quatMsg.z);
    return Pose3(rot, point);
}

geometry_msgs::PoseStamped createStampedPoseMsg(const Pose3 &pose, const ros::Time &stamp) {
    geometry_msgs::PoseStamped stampedMsg;
    stampedMsg.pose = toPoseMsg(pose);
    stampedMsg.header.stamp = stamp;
    return stampedMsg;
}