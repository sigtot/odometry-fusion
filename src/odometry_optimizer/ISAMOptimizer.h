#ifndef ODOMETRY_OPTIMIZER_ISAMOPTIMIZER_H
#define ODOMETRY_OPTIMIZER_ISAMOPTIMIZER_H


#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <gtsam/nonlinear/NonlinearISAM.h>

using namespace gtsam;

class ISAMOptimizer {
private:
    ros::Publisher &pub;
    NonlinearISAM isam;
    NonlinearFactorGraph graph;
public:
    ISAMOptimizer(ros::Publisher *pub, int reorderInterval);
    void recvOdometryAndPublishUpdatedPoses(const nav_msgs::Odometry &msg);
};


#endif //ODOMETRY_OPTIMIZER_ISAMOPTIMIZER_H
