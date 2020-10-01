#ifndef ODOMETRY_OPTIMIZER_UTILS_H
#define ODOMETRY_OPTIMIZER_UTILS_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <gtsam/geometry/Pose3.h>
#include <geometry_msgs/PoseStamped.h>

using namespace gtsam;

geometry_msgs::Pose toPoseMsg(const Pose3 &pose);

Pose3 toPose3(const geometry_msgs::Pose &poseMsg);

geometry_msgs::PoseStamped createStampedPoseMsg(const Pose3 &pose, const ros::Time &stamp);

gtsam::Matrix6 toGtsamMatrix(boost::array<double, 36> arr);

#endif //ODOMETRY_OPTIMIZER_UTILS_H
