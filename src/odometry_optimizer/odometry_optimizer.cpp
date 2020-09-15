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
    ISAMOptimizer isamOptimizer(&pub, 3);
    ros::Subscriber subImu = nh.subscribe("/rovio/odometry", 1000,
                                          &ISAMOptimizer::recvIMUOdometryAndPublishUpdatedPoses, &isamOptimizer);
    ros::Subscriber subLidar = nh.subscribe("/aft_mapped_to_init_CORRECTED", 1000,
                                            &ISAMOptimizer::recvLidarOdometryAndPublishUpdatedPoses, &isamOptimizer);
    ros::spin();
}
