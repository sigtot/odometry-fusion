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
#include "QueuedPoseStampedMeasurementProcessor.h"
#include "IMUQueue.h"
#include "HealthBuffer.h"
#include "Params.h"

using namespace gtsam;
using namespace std;

class ISAMOptimizer : public boost::noncopyable {
private:
    ros::Publisher &pub;
    ros::Publisher &pathPublisher;
    ISAM2 isam;
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

    PoseStampedMeasurement lastOdometryMeasurement;
    PoseStampedMeasurement lastRovioOdometry;
    PoseStampedMeasurement lastLidarOdometry;

    tf::StampedTransform cameraInitTransform;

    QueuedPoseStampedMeasurementProcessor odometryMeasurementProcessor;
    HealthBuffer loamHealthBuffer;
    IMUQueue imuQueue;

    tf::TransformBroadcaster tfBr;

    const double rovioCovariance;
    const double loamCovariance;

    const int extraRovioPriorInterval;

    Params params;
public:
    explicit ISAMOptimizer(ros::Publisher *pub, ros::Publisher *pathPublisher, const boost::shared_ptr<PreintegrationCombinedParams> &imu_params,
                           const tf::StampedTransform &cameraInitTransform, double rovioCovariance,
                           double loamCovariance, int extraRovioPriorInterval, const Params &params);

    void recvIMUMsg(const sensor_msgs::Imu &msg);

    void recvRovioOdometryAndAddToQueue(const nav_msgs::Odometry &msg);

    void recvLidarOdometryAndAddToQueue(const nav_msgs::Odometry &msg);

    bool recvRovioOdometryAndUpdateState(const PoseStampedMeasurement &measurement,
                                         const boost::shared_ptr<noiseModel::Gaussian> &noise);

    bool recvLidarOdometryAndUpdateState(const PoseStampedMeasurement &measurement,
                                         const boost::shared_ptr<noiseModel::Gaussian> &noise);

    bool recvOdometryAndUpdateState(const PoseStampedMeasurement &measurement, int &lastPoseNum, PoseStampedMeasurement &lastOdometry,
                                    const boost::shared_ptr<noiseModel::Gaussian> &noise);

    void recvLoamHealthMsg(const std_msgs::Header &msg);

    void processOdometryMeasurements();

    void processOdometryMeasurement(const PoseStampedMeasurement &measurement);

    void publishUpdatedPoses(ros::Time stamp);

    void publishNewestPose(ros::Time stamp);

    void publishNewestFrame(ros::Time stamp);

    void incrementTime(const ros::Time &stamp);

    NavState getPrevIMUState();

    imuBias::ConstantBias getPrevIMUBias();
};


#endif //ODOMETRY_OPTIMIZER_ISAMOPTIMIZER_H
