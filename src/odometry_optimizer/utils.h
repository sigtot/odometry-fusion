#ifndef ODOMETRY_OPTIMIZER_UTILS_H
#define ODOMETRY_OPTIMIZER_UTILS_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <gtsam/geometry/Pose3.h>
#include <geometry_msgs/PoseStamped.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <sensor_msgs/Imu.h>
#include <deque>

using namespace gtsam;
using namespace std;

geometry_msgs::Pose toPoseMsg(const Pose3 &pose);

Pose3 toPose3(const geometry_msgs::Pose &poseMsg);

gtsam::Matrix6 toGtsamMatrix(boost::array<double, 36> arr);

#endif //ODOMETRY_OPTIMIZER_UTILS_H
