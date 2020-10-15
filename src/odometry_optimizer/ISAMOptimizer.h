#ifndef ODOMETRY_OPTIMIZER_ISAMOPTIMIZER_H
#define ODOMETRY_OPTIMIZER_ISAMOPTIMIZER_H


#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuFactor.h>
#include <boost/noncopyable.hpp>

using namespace gtsam;
using namespace std;

class ISAMOptimizer: public boost::noncopyable {
private:
    ros::Publisher &pub;
    ISAM2 isam;
    NonlinearFactorGraph graph;
    Pose3 lastRovioOdometry;
    Pose3 lastLidarOdometry;
    std::shared_ptr<PreintegrationType> imuMeasurements;
    ros::Time lastIMUTime;
    int imuCount = 0;
    bool imuReady = false;
    NavState prevIMUState;
    imuBias::ConstantBias prevIMUBias;
    vector<ros::Time> timestamps;
    int lastRovioPoseNum = 0;
    int lastLidarPoseNum = 0;
    int lastIMUPoseNum = 0;
    int poseNum = 0;
    mutex mu;
public:
    explicit ISAMOptimizer(ros::Publisher *pub);

    void recvIMUMsgAndUpdateState(const sensor_msgs::Imu &msg);
    void recvIMUAndUpdateState(const Vector3& acc, const Vector3& omega, ros::Time imuTime);

    void recvRovioOdometryMsgAndPublishUpdatedPoses(const nav_msgs::Odometry &msg);
    void recvLidarOdometryMsgAndPublishUpdatedPoses(const nav_msgs::Odometry &msg);
    void recvRovioOdometryAndUpdateState(const Pose3 &odometry, const boost::shared_ptr<noiseModel::Gaussian>& noise);
    void recvLidarOdometryAndUpdateState(const Pose3 &odometry, const boost::shared_ptr<noiseModel::Gaussian>& noise);
    void recvOdometryAndUpdateState(const Pose3 &odometry, int &lastPoseNum, Pose3 &lastOdometry, const boost::shared_ptr<noiseModel::Gaussian>& noise);

    void publishUpdatedPoses();
    void publishNewestPose();
    void incrementTime(const ros::Time &stamp);
    void resetIMUIntegrator();
};


#endif //ODOMETRY_OPTIMIZER_ISAMOPTIMIZER_H
