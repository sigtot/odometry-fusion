#include "ISAMOptimizer.h"
#include "utils.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
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
    auto imu_params = PreintegratedCombinedMeasurements::Params::MakeSharedU(9.81);
    Matrix3 eye3;
    eye3 << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;
    // TODO these should be squared I think
    imu_params->accelerometerCovariance = eye3 * 0.14; // mg/sqrt(Hz)
    imu_params->integrationCovariance = eye3 * 0.001; // don't know
    imu_params->gyroscopeCovariance = eye3 * 0.0035 * 3.14 / 180; // rad/s/sqrt(Hz)
    imu_params->omegaCoriolis = Vector3::Zero(); // don't know
    auto imuBias = imuBias::ConstantBias(); // Initialize at zero bias
    imuMeasurements = std::make_shared<PreintegratedCombinedMeasurements>(imu_params, imuBias);
    Rot3 priorRot = Rot3::Quaternion(1, 0, 0, 0);
    Point3 priorPoint(0, 0, 0);
    Pose3 priorPose(priorRot, priorPoint);
    Vector3 priorVelocity(0, 0, 0);
    prevIMUState = NavState(priorPose, priorVelocity);
    prevIMUBias = imuBias;
    resetIMUIntegrator();
    lastIMUTime = ros::Time(0);
}

void ISAMOptimizer::recvIMUMsgAndUpdateState(const sensor_msgs::Imu &msg) {
    mu.lock();
    auto imuTime = ros::Time(msg.header.stamp.sec, msg.header.stamp.nsec);
    auto linearMsg = msg.linear_acceleration;
    auto acc = Vector3(linearMsg.x, linearMsg.y, linearMsg.z);
    auto angularMsg = msg.angular_velocity;
    auto omega = Vector3(angularMsg.x, angularMsg.y, angularMsg.z);
    recvIMUAndUpdateState(acc, omega, imuTime);
    mu.unlock();
}

void ISAMOptimizer::recvIMUAndUpdateState(const Vector3 &acc, const Vector3 &omega, ros::Time imuTime) {
    if (!imuReady) {
        lastIMUTime = imuTime;
        imuReady = true;
        mu.unlock();
        return;
    }
    if (imuTime < lastIMUTime) {
        // Oops, something is out of order: Just ignore the message until we get something new
        ROS_INFO("Got out of order IMU message");
        mu.unlock();
        return;
    }
    auto dt = imuTime - lastIMUTime;
    imuMeasurements->integrateMeasurement(acc, omega, dt.toSec());
    // add to buffer
    imuCount++;
    lastIMUTime = imuTime;
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

// TODO: Remove
void ISAMOptimizer::incrementTime(const ros::Time &stamp) {
    poseNum++;
    timestamps.push_back(stamp);
}

void ISAMOptimizer::resetIMUIntegrator() {
    imuMeasurements->resetIntegration();
    imuCount = 0;
}

void ISAMOptimizer::recvRovioOdometryMsgAndPublishUpdatedPoses(const nav_msgs::Odometry &msg) {
    mu.lock();
    // using covariance from rovio makes the trajectory all messed up TODO: Don't use it
    recvRovioOdometryAndUpdateState(toPose3(msg.pose.pose),
                                    noiseModel::Gaussian::Covariance(toGtsamMatrix(msg.pose.covariance)));
    publishNewestPose();
    mu.unlock();
}

void ISAMOptimizer::recvLidarOdometryMsgAndPublishUpdatedPoses(const nav_msgs::Odometry &msg) {
    mu.lock();
    recvLidarOdometryAndUpdateState(toPose3(msg.pose.pose),
                                    noiseModel::Diagonal::Variances((Vector(6) << 0.02, 0.02, 0.02, 0.02, 0.02, 0.02).finished()));
    publishNewestPose();
    mu.unlock();
}

void ISAMOptimizer::recvRovioOdometryAndUpdateState(const Pose3 &odometry,
                                                    const boost::shared_ptr<noiseModel::Gaussian> &noise) {
    recvOdometryAndUpdateState(odometry, lastRovioPoseNum, lastRovioOdometry, noise);
}

void ISAMOptimizer::recvLidarOdometryAndUpdateState(const Pose3 &odometry,
                                                    const boost::shared_ptr<noiseModel::Gaussian> &noise) {
    recvOdometryAndUpdateState(odometry, lastLidarPoseNum, lastLidarOdometry, noise);
}

void
ISAMOptimizer::recvOdometryAndUpdateState(const Pose3 &odometry, int &lastPoseNum, Pose3 &lastOdometry,
                                          const boost::shared_ptr<noiseModel::Gaussian> &noise) {
    if (poseNum == 0) {
        // TODO just let this be poseNum == 1 and initialize at t=1 instead. This is just silly
        // We need to add a prior in the first iteration
        auto priorNoiseX = noiseModel::Diagonal::Sigmas((Vector(6) << 0.3, 0.3, 0.3, 0.3, 0.3, 0.3).finished());
        auto priorNoiseV = noiseModel::Isotropic::Sigma(3, 0.1);
        auto priorNoiseB = noiseModel::Isotropic::Sigma(6, 1e-3);

        // TODO join this init step up with the one in the ctor
        graph.add(PriorFactor<Pose3>(X(1), odometry, priorNoiseX)); // Init first pose on rovio or lidar odometry
        graph.addPrior<Pose3>(X(1), odometry, priorNoiseX);
        graph.addPrior<Vector3>(V(1), Vector3::Zero(), priorNoiseV);
        graph.addPrior<imuBias::ConstantBias>(B(1), imuBias::ConstantBias(), priorNoiseB);
        prevIMUState = NavState(odometry, Vector3::Zero()); // Init prev imu state here as well
    }

    poseNum++;

    if (lastPoseNum > 0) {
        auto odometryDelta = lastOdometry.between(odometry);
        graph.add(BetweenFactor<Pose3>(X(lastPoseNum), X(poseNum), odometryDelta, noise));
    }


    Values initialEstimate;
    if (imuCount > 0) {
        if (poseNum > 1) {
            auto imuCombined = dynamic_cast<const PreintegratedCombinedMeasurements &>(*imuMeasurements);
            CombinedImuFactor imuFactor(X(lastIMUPoseNum), V(lastIMUPoseNum), X(poseNum),
                                        V(poseNum), B(lastIMUPoseNum), B(poseNum),
                                        imuCombined);
            graph.add(imuFactor);
        }
        NavState imuState = imuMeasurements->predict(prevIMUState, prevIMUBias);
        initialEstimate.insert(X(poseNum), imuState.pose());
        initialEstimate.insert(V(poseNum), imuState.v());
        initialEstimate.insert(B(poseNum), prevIMUBias);
    } else {
        if (poseNum > 1) {
            initialEstimate.insert(X(poseNum), isam.calculateEstimate(X(poseNum - 1)));
        } else {
            initialEstimate.insert(X(poseNum), odometry); // TODO: Combine with the poseNum == 0 step up above
        }
    }

    isam.update(graph, initialEstimate);
    if (imuCount > 0) {
        prevIMUState = NavState(isam.calculateEstimate<Pose3>(X(poseNum)), isam.calculateEstimate<Vector3>(V(poseNum)));
        prevIMUBias = isam.calculateEstimate<imuBias::ConstantBias>(B(poseNum));
        lastIMUPoseNum = poseNum;
    }
    resetIMUIntegrator();
    lastOdometry = odometry;
    lastPoseNum = poseNum;
    graph.resize(0);
}
