#ifndef ODOMETRY_OPTIMIZER_UTILS_H
#define ODOMETRY_OPTIMIZER_UTILS_H

#include <geometry_msgs/Pose.h>
#include <gtsam/geometry/Pose3.h>

using namespace gtsam;

geometry_msgs::Pose toPoseMsg(const Pose3 &pose);

Pose3 toPose3(const geometry_msgs::Pose &poseMsg);

#endif //ODOMETRY_OPTIMIZER_UTILS_H
