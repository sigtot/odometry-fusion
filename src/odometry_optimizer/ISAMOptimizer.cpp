#include "ISAMOptimizer.h"
#include "utils.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <iostream>

using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

ISAM2Params getParams() {
    ISAM2Params isam2Params;
    isam2Params.relinearizeThreshold = 0.1;
    isam2Params.factorization = ISAM2Params::CHOLESKY;
}

ISAMOptimizer::ISAMOptimizer(ros::Publisher *pub) : pub(*pub),
                                                    isam(ISAM2(ISAM2Params())) {
    auto imu_params = PreintegratedImuMeasurements::Params::MakeSharedU(9.81);
    Matrix3 eye3;
    eye3 << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;
    imu_params->accelerometerCovariance = eye3 * 0.001;
    imu_params->integrationCovariance = eye3 * 0.001;
    imu_params->gyroscopeCovariance = eye3 * 0.001;
    imu_params->omegaCoriolis = Vector3::Zero();
    auto imuBias = imuBias::ConstantBias(); // Assume zero bias
    imuMeasurements = std::make_shared<PreintegratedImuMeasurements>(imu_params, imuBias);
    Rot3 priorRot = Rot3::Quaternion(1, 0, 0, 0);
    Point3 priorPoint(0, 0, 0);
    Pose3 priorPose(priorRot, priorPoint);
    Vector3 priorVelocity(0, 0, 0);
    prevIMUState = NavState(priorPose, priorVelocity);
    resetIMUIntegrator();
    lastIMUTime = ros::Time(0);
}

void ISAMOptimizer::recvIMUAndUpdateState(const sensor_msgs::Imu &msg) {
    mu.lock();
    auto imuTime = ros::Time(msg.header.stamp.sec, msg.header.stamp.nsec);
    if (imuCount == 0) {
        lastIMUTime = imuTime;
        imuCount++;
        mu.unlock();
        return;
    }
    if (imuTime < lastIMUTime) {
        // Oops, something is out of order: Just ignore the message until we get something new
        mu.unlock();
        return;
    }
    auto dt = imuTime - lastIMUTime;
    auto linearMsg = msg.linear_acceleration;
    auto acc = Vector3(linearMsg.x, linearMsg.y, linearMsg.z);
    auto angularMsg = msg.angular_velocity;
    auto omega = Vector3(angularMsg.x, angularMsg.y, angularMsg.z);
    imuMeasurements->integrateMeasurement(acc, omega, dt.toSec());
    // add to buffer
    imuCount++;
    lastIMUTime = imuTime;
    mu.unlock();
}

void ISAMOptimizer::publishUpdatedPoses() {
    nav_msgs::Path pathMsg;
    for (int j = 1; j < poseNum; ++j) {
        auto pose = isam.calculateEstimate<Pose3>(X(j));
        auto stamp = timestamps[j];
        auto stampedPose = createStampedPoseMsg(pose, stamp);
        stampedPose.header.frame_id = "world";
        pathMsg.poses.push_back(stampedPose);
    }
    pathMsg.header.frame_id = "world";
    pub.publish(pathMsg);
    ROS_INFO("Published path");
}

void ISAMOptimizer::publishNewestPose() {
    auto newPose = isam.calculateEstimate<Pose3>(X(poseNum));
    nav_msgs::Odometry msg;
    msg.pose.pose = toPoseMsg(newPose);
    msg.header.frame_id = "world";
    pub.publish(msg);
    ROS_INFO("Published updated pose");
}

void ISAMOptimizer::incrementTime(const ros::Time &stamp) {
    poseNum++;
    timestamps.push_back(stamp);
}

void ISAMOptimizer::resetIMUIntegrator() {
    imuMeasurements->resetIntegration();
}

void ISAMOptimizer::recvRovioOdometryAndPublishUpdatedPoses(const nav_msgs::Odometry &msg) {
    mu.lock();
    recvOdometryAndPublishUpdatedPoses(msg, lastRovioPoseNum, lastRovioOdometry,
                                       noiseModel::Gaussian::Covariance(toGtsamMatrix(msg.pose.covariance)));
    mu.unlock();
}

void ISAMOptimizer::recvLidarOdometryAndPublishUpdatedPoses(const nav_msgs::Odometry &msg) {
    mu.lock();
    recvOdometryAndPublishUpdatedPoses(msg, lastLidarPoseNum, lastLidarOdometry,
                                       noiseModel::Diagonal::Sigmas(Vector6(0.3, 0.3, 0.3, 0.3, 0.3, 0.3)));
    mu.unlock();
}

void
ISAMOptimizer::recvOdometryAndPublishUpdatedPoses(const nav_msgs::Odometry &msg, int &lastPoseNum, Pose3 &lastOdometry,
                                                  const boost::shared_ptr<noiseModel::Gaussian>& noise) {
    auto odometry = toPose3(msg.pose.pose);
    if (poseNum == 0) {
        // We need to add a prior in the first iteration
        auto priorNoise = noiseModel::Diagonal::Sigmas(Vector6(0.3, 0.3, 0.3, 0.3, 0.3, 0.3));
        graph.add(PriorFactor<Pose3>(X(1), odometry, priorNoise)); // Init first pose on rovio odometry
        prevIMUState = NavState(odometry, Vector3::Zero()); // Init prev imu state here as well
    }

    incrementTime(msg.header.stamp);

    NavState imuState = imuMeasurements->predict(prevIMUState, imuBias::ConstantBias());
    if (lastPoseNum > 0) {
        auto odometryDelta = lastOdometry.between(odometry);
        graph.add(BetweenFactor<Pose3>(X(lastPoseNum), X(poseNum), odometryDelta, noise));
    }

    Values initialEstimate;
    initialEstimate.insert(X(poseNum), imuState.pose()); // Initialize on imu measurement.

    if (poseNum > 1) {
        auto imuDelta = imuState.pose().between(prevIMUState.pose());
        auto imuNoise = noiseModel::Diagonal::Sigmas(Vector6(0.002, 0.002, 0.002, 0.002, 0.002, 0.002));
        graph.add(BetweenFactor<Pose3>(X(poseNum - 1), X(poseNum), imuDelta, imuNoise));
    }

    isam.update(graph, initialEstimate);
    prevIMUState = NavState(initialEstimate.at<Pose3>(X(poseNum)), Vector3::Zero());
    publishNewestPose();
    resetIMUIntegrator();
    lastOdometry = odometry;
    lastPoseNum = poseNum;
    graph.resize(0);
}
