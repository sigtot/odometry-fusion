#ifndef ODOMETRY_OPTIMIZER_ISAMOPTIMIZER_H
#define ODOMETRY_OPTIMIZER_ISAMOPTIMIZER_H


#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/geometry/Pose3.h>

using namespace gtsam;
using namespace std;

class ISAMOptimizer {
private:
    ros::Publisher &pub;
    NonlinearISAM isam;
    NonlinearFactorGraph graph;
    int poseNum = 1;
public:
    ISAMOptimizer(ros::Publisher *pub, int reorderInterval);

    void recvOdometryAndPublishUpdatedPoses(const nav_msgs::Odometry &msg);
};


#endif //ODOMETRY_OPTIMIZER_ISAMOPTIMIZER_H
