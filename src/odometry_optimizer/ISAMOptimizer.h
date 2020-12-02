#ifndef ODOMETRY_OPTIMIZER_ISAMOPTIMIZER_H
#define ODOMETRY_OPTIMIZER_ISAMOPTIMIZER_H


#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuFactor.h>
#include <boost/noncopyable.hpp>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include <deque>
#include <thread>
#include <condition_variable>
#include <tf/transform_broadcaster.h>
#include "QueuedOdometryMeasurementProcessor.h"
#include "IMUQueue.h"
#include "HealthBuffer.h"

using namespace gtsam;
using namespace std;

class ISAMOptimizer: public boost::noncopyable {
private:
    ros::Publisher &pub;
    ISAM2 isam;
    Pose3 lastRovioOdometry;
    Pose3 lastLidarOdometry;
    std::shared_ptr<PreintegrationType> imuMeasurements;
    ros::Time lastIMUTime;
    ros::Time lastPoseTime;
    int imuCount = 0;
    bool imuReady = false;
    vector<ros::Time> timestamps;
    int lastRovioPoseNum = 0;
    int lastLidarPoseNum = 0;
    int poseNum = 0;
    mutex mu;
    tf::TransformListener tfListenerNew;
    OdometryMeasurement lastOdometryMeasurement;

    QueuedOdometryMeasurementProcessor odometryMeasurementProcessor;
    HealthBuffer loamHealthBuffer;
    IMUQueue imuQueue;

    tf::TransformBroadcaster tfBr;

    const double rovioCovariance;
    const double loamCovariance;

    const int extraRovioPriorInterval;
public:
    explicit ISAMOptimizer(ros::Publisher *pub, const boost::shared_ptr<PreintegrationCombinedParams>& imu_params, double rovioCovariance, double loamCovariance, int extraRovioPriorInterval);

    void recvIMUMsg(const sensor_msgs::Imu &msg);

    void recvRovioOdometryAndAddToQueue(const nav_msgs::Odometry &msg);
    void recvLidarOdometryAndAddToQueue(const nav_msgs::Odometry &msg);
    bool recvRovioOdometryAndUpdateState(const geometry_msgs::PoseStamped &msg, const boost::shared_ptr<noiseModel::Gaussian>& noise);
    bool recvLidarOdometryAndUpdateState(const geometry_msgs::PoseStamped &msg, const boost::shared_ptr<noiseModel::Gaussian>& noise);
    bool recvOdometryAndUpdateState(const geometry_msgs::PoseStamped &msg, int &lastPoseNum, Pose3 &lastOdometry, const boost::shared_ptr<noiseModel::Gaussian>& noise);

    void recvLoamHealthMsg(const std_msgs::Header &msg);

    void processOdometryMeasurements();
    void processOdometryMeasurement(const OdometryMeasurement &measurement);

    void publishUpdatedPoses();
    void publishNewestPose();
    void publishNewestFrame();
    void incrementTime(const ros::Time &stamp);

    NavState getPrevIMUState();
    imuBias::ConstantBias getPrevIMUBias();
};


#endif //ODOMETRY_OPTIMIZER_ISAMOPTIMIZER_H
