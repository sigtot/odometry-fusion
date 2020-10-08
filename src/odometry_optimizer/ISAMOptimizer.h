#ifndef ODOMETRY_OPTIMIZER_ISAMOPTIMIZER_H
#define ODOMETRY_OPTIMIZER_ISAMOPTIMIZER_H


#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuFactor.h>
#include <boost/noncopyable.hpp>

using namespace gtsam;
using namespace std;

class ISAMOptimizer: public boost::noncopyable {
private:
    ros::Publisher &pub;
    NonlinearISAM isam;
    NonlinearFactorGraph graph;
    Pose3 lastRovioOdometry;
    Pose3 lastLidarOdometry;
    std::shared_ptr<PreintegratedImuMeasurements> imuMeasurements;
    ros::Time lastIMUTime;
    int imuCount = 0;
    NavState prevIMUState;
    vector<ros::Time> timestamps;
    int lastLidarPoseNum = 1;
    int poseNum = 0;
    mutex mu;
public:
    ISAMOptimizer(ros::Publisher *pub, int reorderInterval);

    void recvIMUAndUpdateState(const sensor_msgs::Imu &msg);
    void recvRovioOdometryAndPublishUpdatedPoses(const nav_msgs::Odometry &msg);
    void recvLidarOdometryAndPublishUpdatedPoses(const nav_msgs::Odometry &msg);
    void publishUpdatedPoses();
    void publishNewestPose();
    void incrementTime(const ros::Time &stamp);
    void resetIMUIntegrator();
};


#endif //ODOMETRY_OPTIMIZER_ISAMOPTIMIZER_H
