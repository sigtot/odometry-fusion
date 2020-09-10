#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include "ISAMOptimizer.h"

using namespace std;
using namespace gtsam;

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometry_optimizer");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Pose>("/optimized_pose", 1000);
    auto isamOptimizer = ISAMOptimizer(&pub, 3);
    ros::Subscriber sub = nh.subscribe("/rovio/odometry", 1000, &ISAMOptimizer::recvOdometryAndPublishUpdatedPoses,
                                       &isamOptimizer);
    ros::spin();
}
