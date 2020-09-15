#ifndef ODOMETRY_OPTIMIZER_ISAMOPTIMIZER_H
#define ODOMETRY_OPTIMIZER_ISAMOPTIMIZER_H


#include <mutex>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/geometry/Pose3.h>
#include <boost/noncopyable.hpp>

using namespace gtsam;
using namespace std;

class ISAMOptimizer: public boost::noncopyable {
private:
    ros::Publisher &pub;
    NonlinearISAM isam;
    NonlinearFactorGraph graph;
    Pose3 lastIMUOdometry;
    Pose3 lastLidarOdometry;
    vector<ros::Time> timestamps;
    int lastLidarPoseNum = 1;
    int poseNum = 0;
    mutex mu;
public:
    ISAMOptimizer(ros::Publisher *pub, int reorderInterval);

    void recvIMUOdometryAndPublishUpdatedPoses(const nav_msgs::Odometry &msg);
    void recvLidarOdometryAndPublishUpdatedPoses(const nav_msgs::Odometry &msg);
    void publishUpdatedPoses();
    void incrementTime(const ros::Time &stamp);
};


#endif //ODOMETRY_OPTIMIZER_ISAMOPTIMIZER_H
