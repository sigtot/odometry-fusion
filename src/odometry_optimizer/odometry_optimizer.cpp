#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "ISAMOptimizer.h"

using namespace std;
using namespace gtsam;

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometry_optimizer");
    ros::NodeHandle nh;
    auto pub = nh.advertise<nav_msgs::Odometry>("/optimized_pose", 1000);
    ISAMOptimizer isamOptimizer(&pub, 3);
    ros::Subscriber subImu = nh.subscribe("/rovio/odometry", 1000,
                                          &ISAMOptimizer::recvRovioOdometryAndPublishUpdatedPoses, &isamOptimizer);
    ros::Subscriber subLidar = nh.subscribe("/aft_mapped_to_init_CORRECTED", 1000,
                                            &ISAMOptimizer::recvLidarOdometryAndPublishUpdatedPoses, &isamOptimizer);
    ros::spin();
}
