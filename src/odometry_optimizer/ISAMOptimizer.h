#ifndef ODOMETRY_OPTIMIZER_ISAMOPTIMIZER_H
#define ODOMETRY_OPTIMIZER_ISAMOPTIMIZER_H


#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

class ISAMOptimizer {
private:
    ros::Publisher &pub;
public:
    explicit ISAMOptimizer(ros::Publisher *pub);
    void recvOdometryAndPublishUpdatedPoses(const nav_msgs::Odometry &msg);
};


#endif //ODOMETRY_OPTIMIZER_ISAMOPTIMIZER_H
