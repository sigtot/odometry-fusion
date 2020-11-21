#include "ISAMOptimizer.h"
#include "utils.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <iostream>

#include "geometry_msgs/PoseStamped.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

ISAM2Params getParams() {
    ISAM2Params isam2Params;
    isam2Params.relinearizeThreshold = 0.1;
    isam2Params.factorization = ISAM2Params::CHOLESKY;
}

ISAMOptimizer::ISAMOptimizer(ros::Publisher *pub) : pub(*pub), isam(ISAM2(ISAM2Params())), tfListener(tfBuffer) {
    auto imu_params = PreintegratedCombinedMeasurements::Params::MakeSharedU(9.81);
    Matrix3 eye3;
    eye3 << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;
    // TODO these should be squared I think
    imu_params->accelerometerCovariance = eye3 * 0.14; // mg/sqrt(Hz)
    imu_params->integrationCovariance = eye3 * 0; // Kitty is zero
    imu_params->gyroscopeCovariance = eye3 * 0.0035 * 3.14 / 180; // rad/s/sqrt(Hz)
    imu_params->omegaCoriolis = Vector3::Zero(); // don't know
    auto imuBias = imuBias::ConstantBias(); // Initialize at zero bias
    imuMeasurements = std::make_shared<PreintegratedCombinedMeasurements>(imu_params, imuBias);
    resetIMUIntegrator();
}

ISAMOptimizer::ISAMOptimizer(ros::Publisher *pub, const boost::shared_ptr<PreintegrationCombinedParams>& imu_params) : pub(*pub), isam(ISAM2(ISAM2Params())), tfListener(tfBuffer) {
    auto imuBias = imuBias::ConstantBias(); // Initialize at zero bias
    imuMeasurements = std::make_shared<PreintegratedCombinedMeasurements>(imu_params, imuBias);
    resetIMUIntegrator();
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
    bool shouldPublish = recvRovioOdometryAndUpdateState(toPose3(msg.pose.pose), noiseModel::Diagonal::Variances(
            (Vector(6) << 0.05, 0.05, 0.05, 0.05, 0.05, 0.05).finished()));
    if (shouldPublish) {
        publishNewestPose();
    }
    mu.unlock();
}

void ISAMOptimizer::recvLidarOdometryMsgAndPublishUpdatedPoses(const nav_msgs::Odometry &msg) {
    mu.lock();
    cout << "Can transform camera_init -> world : " << tfBuffer.canTransform("camera_init", "world", ros::Time(0)) << endl;
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header = msg.header;
    poseStamped.pose = msg.pose.pose;
    cout << "Frame of pose" << poseStamped.header.frame_id << endl;
    auto poseMsgInWorldFrame = tfBuffer.transform(poseStamped, "world");
    bool shouldPublish = recvLidarOdometryAndUpdateState(toPose3(poseMsgInWorldFrame.pose), noiseModel::Diagonal::Variances(
            (Vector(6) << 0.2, 0.2, 0.2, 0.2, 0.2, 0.2).finished()));
    if (shouldPublish) {
        publishNewestPose();
    }
    mu.unlock();
}

bool ISAMOptimizer::recvRovioOdometryAndUpdateState(const Pose3 &odometry,
                                                    const boost::shared_ptr<noiseModel::Gaussian> &noise) {
    return recvOdometryAndUpdateState(odometry, lastRovioPoseNum, lastRovioOdometry, noise);
}

bool ISAMOptimizer::recvLidarOdometryAndUpdateState(const Pose3 &odometry,
                                                    const boost::shared_ptr<noiseModel::Gaussian> &noise) {
    return recvOdometryAndUpdateState(odometry, lastLidarPoseNum, lastLidarOdometry, noise);
}

void
addPoseVelocityAndBiasValues(int poseNum, const Pose3 &pose, const Vector3 &velocity, const imuBias::ConstantBias &bias,
                             Values &values) {
    values.insert(X(poseNum), pose);
    values.insert(V(poseNum), velocity);
    values.insert(B(poseNum), bias);
}

void addValuesOnIMU(int poseNum, const NavState &navState, const imuBias::ConstantBias &bias, Values &values) {
    addPoseVelocityAndBiasValues(poseNum, navState.pose(), navState.v(), bias, values);
}

void addValuesOnOdometry(int poseNum, const Pose3 &odometry, const Vector3 &velocity, const imuBias::ConstantBias &bias, Values &values) {
    addPoseVelocityAndBiasValues(poseNum, odometry, velocity, bias, values);
}

void addOdometryBetweenFactor(int fromPoseNum, int poseNum, const Pose3 &fromOdometry, const Pose3 &odometry,
                              const boost::shared_ptr<noiseModel::Gaussian> &noise, NonlinearFactorGraph &graph) {
    auto odometryDelta = fromOdometry.between(odometry);
    graph.add(BetweenFactor<Pose3>(X(fromPoseNum), X(poseNum), odometryDelta, noise));
}

void addPriorFactor(const Pose3 &pose, const Vector3 &velocity, const imuBias::ConstantBias &bias,
                    const boost::shared_ptr<noiseModel::Diagonal> &noiseX,
                    const boost::shared_ptr<noiseModel::Isotropic> &noiseV,
                    const boost::shared_ptr<noiseModel::Isotropic> &noiseB, NonlinearFactorGraph &graph) {
    graph.addPrior<Pose3>(X(1), pose, noiseX);
    graph.addPrior<Vector3>(V(1), velocity, noiseV);
    graph.addPrior<imuBias::ConstantBias>(B(1), bias, noiseB);
}

void addCombinedFactor(int poseNum, const Pose3 &fromOdometry, const Pose3 &odometry,
                       const shared_ptr<TangentPreintegration> &imuMeasurements,
                       const boost::shared_ptr<noiseModel::Gaussian> &odometryNoise, NonlinearFactorGraph &graph) {
    auto odometryDelta = fromOdometry.between(odometry);
    auto imuCombined = dynamic_cast<const PreintegratedCombinedMeasurements &>(*imuMeasurements);
    CombinedImuFactor imuFactor(X(poseNum - 1), V(poseNum - 1), X(poseNum), V(poseNum), B(poseNum - 1), B(poseNum),
                                imuCombined);
    graph.add(imuFactor);
}

bool
ISAMOptimizer::recvOdometryAndUpdateState(const Pose3 &odometry, int &lastPoseNum, Pose3 &lastOdometry,
                                          const boost::shared_ptr<noiseModel::Gaussian> &noise) {
    Values values;
    NonlinearFactorGraph graph;
    if (poseNum > 0 && imuCount > 1) {
        poseNum++;
        if (lastPoseNum > 0) {
            addOdometryBetweenFactor(lastPoseNum, poseNum, lastOdometry, odometry, noise, graph);
        }
        addCombinedFactor(poseNum, lastOdometry, odometry, imuMeasurements, noise, graph);
        NavState navState = imuMeasurements->predict(getPrevIMUState(), getPrevIMUBias());
        addValuesOnIMU(poseNum, navState, getPrevIMUBias(), values);
        isam.update(graph, values);
        resetIMUIntegrator();
        lastOdometry = odometry;
        lastPoseNum = poseNum;
        return true;
    }
    if (poseNum == 0 && imuCount > 1) {
        poseNum++;
        auto priorNoiseX = noiseModel::Diagonal::Sigmas((Vector(6) << 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001).finished()); // We are dead sure about starting pos
        auto priorNoiseV = noiseModel::Isotropic::Sigma(3, 0.1);
        auto priorNoiseB = noiseModel::Isotropic::Sigma(6, 2);
        addPriorFactor(odometry, Vector3::Zero(), imuBias::ConstantBias(), priorNoiseX, priorNoiseV, priorNoiseB, graph);
        addPoseVelocityAndBiasValues(poseNum, odometry, Vector3::Zero(), imuBias::ConstantBias(), values);
        isam.update(graph, values);
        resetIMUIntegrator();
        lastOdometry = odometry;
        lastPoseNum = poseNum;
        return true;
    }
    if (poseNum > 0 && imuCount == 0) {
        // We do not increment poseNum because the time interval since last pose is so small
        cout << "Not enough time between imu measurements: " << lastPoseNum << endl;
        if (lastPoseNum > 0 && poseNum  > lastPoseNum) {
            cout << "Adding between factor without imu" << lastPoseNum << endl;
            addOdometryBetweenFactor(lastPoseNum, poseNum, lastOdometry, odometry, noise, graph);
            isam.update(graph, values);
            lastPoseNum = poseNum;
            lastOdometry = odometry;
        }
        return false;
    }
    if (poseNum == 0 && imuCount == 0) {
        // Skip odometry measurements until we have IMU
        return false;
    }
}

NavState ISAMOptimizer::getPrevIMUState() {
    return NavState(isam.calculateEstimate<Pose3>(X(poseNum - 1)), isam.calculateEstimate<Vector3>(V(poseNum - 1)));
}

imuBias::ConstantBias ISAMOptimizer::getPrevIMUBias() {
    return isam.calculateEstimate<imuBias::ConstantBias>(B(poseNum - 1));
}
