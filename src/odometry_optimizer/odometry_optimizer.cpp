#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "ISAMOptimizer.h"
#include <iostream>

using namespace std;
using namespace gtsam;

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometry_optimizer");
    ros::NodeHandle nh;
    auto pub = nh.advertise<nav_msgs::Odometry>("/optimized_pose", 1000);
    ISAMOptimizer isamOptimizer(&pub);
    std::string imuTopicName, odometry1TopicName, odometry2TopicName;

    if (!nh.getParam("imu_topic_name", imuTopicName)) {
        cout << "Please supply a value for imu_topic_name" << endl;
    }
    nh.getParam("odometry1_topic_name", odometry1TopicName);
    nh.getParam("odometry2_topic_name", odometry2TopicName);
    ros::Subscriber subIMU = nh.subscribe(imuTopicName,
                                          1000,
                                          &ISAMOptimizer::recvIMUMsgAndUpdateState,
                                          &isamOptimizer);
    ros::Subscriber subRovio = nh.subscribe(odometry1TopicName,
                                            1000,
                                            &ISAMOptimizer::recvRovioOdometryMsgAndPublishUpdatedPoses,
                                            &isamOptimizer);
    ros::Subscriber subLidar = nh.subscribe(odometry2TopicName,
                                            1000,
                                            &ISAMOptimizer::recvLidarOdometryMsgAndPublishUpdatedPoses,
                                            &isamOptimizer);
    ros::spin();
}
