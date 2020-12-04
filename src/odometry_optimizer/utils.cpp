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
    quatMsg.w = pose.rotation().toQuaternion().w();
    quatMsg.x = pose.rotation().toQuaternion().x();
    quatMsg.y = pose.rotation().toQuaternion().y();
    quatMsg.z = pose.rotation().toQuaternion().z();

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

// Silly conversion function while I look for a better way to convert an array to an eigen::Matrix
gtsam::Matrix6 toGtsamMatrix(boost::array<double, 36> arr) {
    Matrix6 mat;
    auto ait = arr.begin();
    int j = 0;
    while (ait != arr.end()) {
        mat(j / 6, j % 6) = *ait;
        j++;
        ait++;
    }
    return mat;
}

