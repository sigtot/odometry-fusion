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
#include <gtsam/navigation/GPSFactor.h>

#include "geometry_msgs/PoseStamped.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <chrono>
#include <thread>

using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

ISAM2Params getParams() {
    ISAM2Params isam2Params;
    isam2Params.relinearizeThreshold = 0.1;
    isam2Params.factorization = ISAM2Params::CHOLESKY;
}

ISAMOptimizer::ISAMOptimizer(ros::Publisher *pub, const boost::shared_ptr<PreintegrationCombinedParams> &imu_params,
                             const tf::StampedTransform &cameraInitTransform, double rovioCovariance,
                             double loamCovariance, int extraRovioPriorInterval)
        : pub(*pub),
          isam(ISAM2(ISAM2Params())),
          cameraInitTransform(cameraInitTransform),
          rovioCovariance(rovioCovariance),
          loamCovariance(loamCovariance),
          extraRovioPriorInterval(extraRovioPriorInterval),
          loamHealthBuffer(15, 3),
          odometryMeasurementProcessor(std::bind(&ISAMOptimizer::processOdometryMeasurement,
                                                 this,
                                                 std::placeholders::_1),
                                       4) {
    cout << "Starting up with rovio covariance " << rovioCovariance << " and loam covariance " << loamCovariance
         << endl;
    imu_params->print("IMU params: ");
    auto imuBias = imuBias::ConstantBias(); // Initialize at zero bias
    imuMeasurements = std::make_shared<PreintegratedCombinedMeasurements>(imu_params, imuBias);
    imuMeasurements->resetIntegration();
}

void ISAMOptimizer::recvIMUMsg(const sensor_msgs::Imu &msg) {
    imuQueue.addMeasurement(msg);
}

void ISAMOptimizer::publishUpdatedPoses() {
    nav_msgs::Path pathMsg;
    for (int j = 1; j < poseNum; ++j) {
        auto pose = isam.calculateEstimate<Pose3>(X(j));
        auto stamp = timestamps[j];
        auto stampedPose = createStampedPoseMsg(pose, stamp);
        stampedPose.header.frame_id = "/map";
        pathMsg.poses.push_back(stampedPose);
    }
    pathMsg.header.frame_id = "/map";
    pub.publish(pathMsg);
    ROS_INFO("Published path");
}

void ISAMOptimizer::publishNewestPose() {
    auto newPose = isam.calculateEstimate<Pose3>(X(poseNum));
    nav_msgs::Odometry msg;
    msg.pose.pose = toPoseMsg(newPose);
    msg.header.frame_id = "/map";
    msg.header.stamp = ros::Time::now();
    pub.publish(msg);
}

void ISAMOptimizer::publishNewestFrame() {
    auto newPose = isam.calculateEstimate<Pose3>(X(poseNum));
    auto poseQuat = newPose.rotation().toQuaternion();
    auto poseTrans = newPose.translation();
    tf::Transform transform;
    tf::Quaternion tfQuat(poseQuat.x(), poseQuat.y(), poseQuat.z(), poseQuat.w());
    tf::Vector3 tfOrigin(poseTrans.x(), poseTrans.y(), poseTrans.z());

    transform.setRotation(tfQuat.normalized());
    transform.setOrigin(tfOrigin);

    tfBr.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/velodyne_fused"));
}

// TODO: Remove
void ISAMOptimizer::incrementTime(const ros::Time &stamp) {
    poseNum++;
    timestamps.push_back(stamp);
}

void ISAMOptimizer::processOdometryMeasurement(const PoseStampedMeasurement &measurement) {
    mu.lock();
    bool shouldPublish = false;
    switch (measurement.type) {
        case ODOMETRY_TYPE_ROVIO:
            shouldPublish = recvRovioOdometryAndUpdateState(measurement, noiseModel::Diagonal::Variances(
                    (Vector(6)
                            << rovioCovariance, rovioCovariance, rovioCovariance, rovioCovariance, rovioCovariance, rovioCovariance).finished()));
            break;
        case ODOMETRY_TYPE_LOAM:
            if (loamHealthBuffer.isHealthy(measurement.msg.header.stamp)) {
                PoseStampedMeasurement transformedMeasurement = measurement;
                tfListenerNew.transformPose("/map", measurement.msg,
                                            transformedMeasurement.msg); // Can fail if newer transform message has arrived
                shouldPublish = recvLidarOdometryAndUpdateState(measurement, noiseModel::Diagonal::Variances(
                        (Vector(6)
                                << loamCovariance, loamCovariance, loamCovariance, loamCovariance, loamCovariance, loamCovariance).finished()));
            }
            break;
    }
    if (shouldPublish) {
        publishNewestFrame();
        publishNewestPose();
    }
    lastOdometryMeasurement = measurement;
    mu.unlock();
}

void ISAMOptimizer::recvRovioOdometryAndAddToQueue(const nav_msgs::Odometry &msg) {
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header = msg.header;
    poseStamped.pose = msg.pose.pose;
    PoseStampedMeasurement measurement{ODOMETRY_TYPE_ROVIO, poseStamped};
    odometryMeasurementProcessor.addMeasurement(measurement);
}

void ISAMOptimizer::recvLidarOdometryAndAddToQueue(const nav_msgs::Odometry &msg) {
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header = msg.header;
    poseStamped.pose = msg.pose.pose;
    PoseStampedMeasurement measurement{ODOMETRY_TYPE_LOAM, poseStamped};
    odometryMeasurementProcessor.addMeasurement(measurement);
}

bool ISAMOptimizer::recvRovioOdometryAndUpdateState(const PoseStampedMeasurement &measurement,
                                                    const boost::shared_ptr<noiseModel::Gaussian> &noise) {
    return recvOdometryAndUpdateState(measurement, lastRovioPoseNum, lastRovioOdometry, noise);
}

bool ISAMOptimizer::recvLidarOdometryAndUpdateState(const PoseStampedMeasurement &measurement,
                                                    const boost::shared_ptr<noiseModel::Gaussian> &noise) {
    return recvOdometryAndUpdateState(measurement, lastLidarPoseNum, lastLidarOdometry, noise);
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

void addValuesOnOdometry(int poseNum, const Pose3 &odometry, const Vector3 &velocity, const imuBias::ConstantBias &bias,
                         Values &values) {
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
ISAMOptimizer::recvOdometryAndUpdateState(const PoseStampedMeasurement &measurement, int &lastPoseNum,
                                          PoseStampedMeasurement &lastOdometry, const boost::shared_ptr<noiseModel::Gaussian> &noise) {
    Values values;
    NonlinearFactorGraph graph;
    auto odometry = toPose3(measurement.msg.pose);
    if (poseNum == 0) {
        poseNum++;
        auto priorNoiseX = noiseModel::Diagonal::Sigmas((Vector(6)
                << 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001).finished()); // We are dead sure about starting pos
        auto priorNoiseV = noiseModel::Isotropic::Sigma(3, 0.01);
        auto priorNoiseB = noiseModel::Isotropic::Sigma(6, 1);
        addPriorFactor(odometry, Vector3::Zero(), imuBias::ConstantBias(), priorNoiseX, priorNoiseV, priorNoiseB,
                       graph);
        addPoseVelocityAndBiasValues(poseNum, odometry, Vector3::Zero(), imuBias::ConstantBias(), values);
        isam.update(graph, values);
        imuMeasurements->resetIntegration(); // TODO Remove
        lastOdometry = measurement;
        lastPoseNum = poseNum;
        return true;
    }
    auto haveIMUMeasurements = imuQueue.hasMeasurementsInRange(lastOdometryMeasurement.msg.header.stamp,
                                                               measurement.msg.header.stamp);
    if (haveIMUMeasurements) {
        poseNum++;
        if (lastPoseNum > 0) {
            addOdometryBetweenFactor(lastPoseNum, poseNum, toPose3(lastOdometry.msg.pose), odometry, noise, graph);
            if (extraRovioPriorInterval != 0 && poseNum % extraRovioPriorInterval == 0 &&
                measurement.msg.header.frame_id != "/map") {
                auto extraPriorNoise = noiseModel::Diagonal::Variances((Vector(6)
                        << rovioCovariance, rovioCovariance, rovioCovariance, rovioCovariance, rovioCovariance, rovioCovariance).finished());
                cout << "Adding prior on " << X(poseNum) << " from msg in frame " << measurement.msg.header.frame_id
                     << endl;
                graph.addPrior<Pose3>(X(poseNum), odometry, extraPriorNoise);
            }
        } else {
            // If we have initialized the other modality, but not this one, we need to add a second hard prior on the pose
            auto priorNoiseX = noiseModel::Diagonal::Sigmas(
                    (Vector(6) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished());
            graph.addPrior<Pose3>(X(poseNum), odometry, priorNoiseX);
            cout << "Added prior on second modality" << endl;
        }
        imuQueue.integrateIMUMeasurements(imuMeasurements, lastOdometryMeasurement.msg.header.stamp,
                                          measurement.msg.header.stamp);
        addCombinedFactor(poseNum, toPose3(lastOdometry.msg.pose), odometry, imuMeasurements, noise, graph);
        NavState navState = imuMeasurements->predict(getPrevIMUState(), getPrevIMUBias());
        addValuesOnIMU(poseNum, navState, getPrevIMUBias(), values);
        isam.update(graph, values);
        auto bias = isam.calculateEstimate<imuBias::ConstantBias>(B(poseNum));
        imuMeasurements->resetIntegrationAndSetBias(bias);
        lastOdometry = measurement;
        lastPoseNum = poseNum;
        return true;
    } else {
        // We do not increment poseNum because the time interval since last pose is so small
        cout << "Not enough time between odometry measurements: " << lastPoseNum << endl;
        if (lastPoseNum > 0 && poseNum > lastPoseNum) {
            cout << "Adding between factor without imu" << lastPoseNum << endl;
            addOdometryBetweenFactor(lastPoseNum, poseNum, toPose3(lastOdometry.msg.pose), odometry, noise, graph);
            isam.update(graph, values);
            lastPoseNum = poseNum;
            lastOdometry = measurement;
        }
        return false;
    }
}

NavState ISAMOptimizer::getPrevIMUState() {
    return NavState(isam.calculateEstimate<Pose3>(X(poseNum - 1)), isam.calculateEstimate<Vector3>(V(poseNum - 1)));
}

imuBias::ConstantBias ISAMOptimizer::getPrevIMUBias() {
    return isam.calculateEstimate<imuBias::ConstantBias>(B(poseNum - 1));
}

void ISAMOptimizer::recvLoamHealthMsg(const std_msgs::Header &msg) {
    loamHealthBuffer.addMeasurement(msg);
}
