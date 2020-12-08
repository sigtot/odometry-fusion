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

ISAM2Params getIsam2Params() {
    ISAM2Params isam2Params;
    isam2Params.relinearizeThreshold = 0.1;
    isam2Params.factorization = ISAM2Params::QR;
    return isam2Params;
}

ISAMOptimizer::ISAMOptimizer(ros::Publisher *pub, ros::Publisher *pathPublisher,
                             const boost::shared_ptr<PreintegrationCombinedParams> &imu_params,
                             const tf::StampedTransform &cameraInitTransform, double rovioCovariance,
                             double loamCovariance, int extraRovioPriorInterval, const Params &params)
        : pub(*pub),
          pathPublisher(*pathPublisher),
          isam(ISAM2(getIsam2Params())),
          params(params),
          cameraInitTransform(cameraInitTransform),
          rovioCovariance(rovioCovariance),
          loamCovariance(loamCovariance),
          extraRovioPriorInterval(extraRovioPriorInterval),
          loamHealthBuffer(30, 3),
          imuQueue(params.imuQueueParams),
          odometryMeasurementProcessor(std::bind(&ISAMOptimizer::processOdometryMeasurement,
                                                 this,
                                                 std::placeholders::_1),
                                       6) {
    cout << "Starting up with rovio covariance " << rovioCovariance << " and loam covariance " << loamCovariance
         << endl;
    imu_params->print("IMU params: ");
    cout << "Only imu: " << params.onlyIMU << endl;
    auto imuBias = imuBias::ConstantBias(); // Initialize at zero bias
    imuMeasurements = std::make_shared<PreintegratedCombinedMeasurements>(imu_params, imuBias);
    imuMeasurements->resetIntegration();
}

void ISAMOptimizer::recvIMUMsg(const sensor_msgs::Imu &msg) {
    imuQueue.addMeasurement(msg);
}

void ISAMOptimizer::publishUpdatedPoses(ros::Time stamp) {
    nav_msgs::Path pathMsg;
    for (int j = 1; j < poseNum; ++j) {
        auto pose = isam.calculateEstimate<Pose3>(X(j));
        auto stamp = timestamps[j];
        geometry_msgs::PoseStamped stampedPoseMsg;
        stampedPoseMsg.pose = toPoseMsg(pose);
        stampedPoseMsg.header.stamp = stamp;
        stampedPoseMsg.header.frame_id = "/map";
        pathMsg.poses.push_back(stampedPoseMsg);
    }
    pathMsg.header.frame_id = "/map";
    pathMsg.header.stamp = stamp;
    pathPublisher.publish(pathMsg);
}

void ISAMOptimizer::publishNewestPose(ros::Time stamp) {
    auto newPose = isam.calculateEstimate<Pose3>(X(poseNum));
    nav_msgs::Odometry msg;
    msg.pose.pose = toPoseMsg(newPose);
    msg.header.frame_id = "/map";
    msg.header.stamp = stamp;
    pub.publish(msg);
}

void ISAMOptimizer::publishNewestFrame(ros::Time stamp) {
    auto newPose = isam.calculateEstimate<Pose3>(X(poseNum));
    auto poseQuat = newPose.rotation().toQuaternion();
    auto poseTrans = newPose.translation();
    tf::Transform transform;
    tf::Quaternion tfQuat(poseQuat.x(), poseQuat.y(), poseQuat.z(), poseQuat.w());
    tf::Vector3 tfOrigin(poseTrans.x(), poseTrans.y(), poseTrans.z());

    transform.setRotation(tfQuat.normalized());
    transform.setOrigin(tfOrigin);

    tfBr.sendTransform(tf::StampedTransform(transform, stamp, "/map", "/velodyne_fused"));
}

// TODO: Remove
void ISAMOptimizer::incrementTime(const ros::Time &stamp) {
    poseNum++;
    timestamps.push_back(stamp);
}

void ISAMOptimizer::processOdometryMeasurement(const PoseStampedMeasurement &measurement) {
    mu.lock();
    bool shouldPublish = false;
    bool haveIMUMeasurements = imuQueue.hasMeasurementsInRange(lastOdometryMeasurement.msg.header.stamp,
                                                               measurement.msg.header.stamp);
    switch (measurement.type) {
        case ODOMETRY_TYPE_ROVIO: {
            shouldPublish = recvRovioOdometryAndUpdateState(measurement, noiseModel::Diagonal::Variances(
                    (Vector(6)
                            << rovioCovariance, rovioCovariance, rovioCovariance, rovioCovariance, rovioCovariance, rovioCovariance).finished()));
            break;
        }
        case ODOMETRY_TYPE_LOAM: {
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
    }
    if (shouldPublish) {
        publishNewestFrame(measurement.msg.header.stamp);
        publishNewestPose(measurement.msg.header.stamp);
        publishUpdatedPoses(measurement.msg.header.stamp);
    }
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

void
addCombinedFactor(int poseNum, const shared_ptr<TangentPreintegration> &imuMeasurements, NonlinearFactorGraph &graph) {
    auto imuCombined = dynamic_cast<const PreintegratedCombinedMeasurements &>(*imuMeasurements);
    CombinedImuFactor imuFactor(X(poseNum - 1), V(poseNum - 1), X(poseNum), V(poseNum), B(poseNum - 1), B(poseNum),
                                imuCombined);
    graph.add(imuFactor);
}

bool
ISAMOptimizer::recvOdometryAndUpdateState(const PoseStampedMeasurement &measurement, int &lastPoseNum,
                                          PoseStampedMeasurement &lastOdometry,
                                          const boost::shared_ptr<noiseModel::Gaussian> &noise) {
    Values values;
    NonlinearFactorGraph graph;
    auto odometry = toPose3(measurement.msg.pose);
    if (poseNum == 0) {
        poseNum++;
        timestamps.push_back(measurement.msg.header.stamp);
        auto priorNoiseX = noiseModel::Diagonal::Sigmas((Vector(6)
                << 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001).finished()); // We are dead sure about starting pos
        auto priorNoiseV = noiseModel::Isotropic::Sigma(3, 0.01);
        auto priorNoiseB = noiseModel::Isotropic::Sigma(6, 1);
        imuBias::ConstantBias initialBias(Vector3(params.initBiasX, params.initBiasY, params.initBiasZ),
                                          Vector3(params.initBiasGX, params.initBiasGY, params.initBiasGZ));
        addPriorFactor(odometry, Vector3::Zero(), initialBias, priorNoiseX, priorNoiseV, priorNoiseB,
                       graph);
        addPoseVelocityAndBiasValues(poseNum, odometry, Vector3::Zero(), initialBias, values);
        isam.update(graph, values);
        lastOdometry = measurement;
        lastOdometryMeasurement = measurement;
        lastPoseNum = poseNum;
        return true;
    }
    auto haveIMUMeasurements = imuQueue.hasMeasurementsInRange(lastOdometryMeasurement.msg.header.stamp,
                                                               measurement.msg.header.stamp);
    if (haveIMUMeasurements) {
        poseNum++;
        timestamps.push_back(measurement.msg.header.stamp);
        if (lastPoseNum > 0 && (measurement.type == ODOMETRY_TYPE_ROVIO ||
                                (loamHealthBuffer.isHealthy(lastLidarOdometry.msg.header.stamp) &&
                                 !loamHealthBuffer.wasDegenerateLastTimeStep()))) {
            if (!params.onlyIMU) {
                addOdometryBetweenFactor(lastPoseNum, poseNum, toPose3(lastOdometry.msg.pose), odometry, noise, graph);
            }
            if (extraRovioPriorInterval != 0 && poseNum % extraRovioPriorInterval == 0 &&
                measurement.type == ODOMETRY_TYPE_ROVIO) {
                auto extraPriorNoise = noiseModel::Diagonal::Variances((Vector(6)
                        << rovioCovariance, rovioCovariance, rovioCovariance, rovioCovariance, rovioCovariance, rovioCovariance).finished());
                cout << "Adding prior on " << X(poseNum) << " from msg in frame " << measurement.msg.header.frame_id
                     << endl;
                graph.addPrior<Pose3>(X(poseNum), odometry, extraPriorNoise);
            }
        }
        imuQueue.integrateIMUMeasurements(imuMeasurements, lastOdometryMeasurement.msg.header.stamp,
                                          measurement.msg.header.stamp);
        addCombinedFactor(poseNum, imuMeasurements, graph);
        NavState navState = imuMeasurements->predict(getPrevIMUState(), getPrevIMUBias());
        addValuesOnIMU(poseNum, navState, getPrevIMUBias(), values);
        isam.update(graph, values);
        auto bias = isam.calculateEstimate<imuBias::ConstantBias>(B(poseNum));

        /*
        auto odometryDelta = toPose3(lastOdometryMeasurement.msg.pose).between(odometry);


        auto velocity = isam.calculateEstimate<Vector3>(V(poseNum));
        auto predictedIMUNavState = imuMeasurements->predict(NavState(Pose3(), velocity), imuBias::ConstantBias());
        cout << "velocity" << velocity << endl;
        cout << "imu delta norm: " << predictedIMUNavState.pose().translation().norm() << " ["
             << predictedIMUNavState.pose().translation() << "]" << endl;
        cout << "odo delta norm: " << odometryDelta.translation().norm() << " [" << odometryDelta.translation() << "]"
             << endl;

         */

        imuMeasurements->resetIntegrationAndSetBias(bias);
        lastOdometry = measurement;
        lastOdometryMeasurement = measurement;
        lastPoseNum = poseNum;

        return true;
    } else {
        // We do not increment poseNum because the time interval since last pose is so small
        cout << "Not enough time between odometry measurements: " << lastPoseNum << endl;
        // This really needs refactoring...
        if (lastPoseNum > 0 && poseNum > lastPoseNum && (measurement.type == ODOMETRY_TYPE_ROVIO ||
                                                         (loamHealthBuffer.isHealthy(
                                                                 lastLidarOdometry.msg.header.stamp) &&
                                                          !loamHealthBuffer.wasDegenerateLastTimeStep()))) {
            if (!params.onlyIMU) {
                cout << "Adding between factor without imu" << lastPoseNum << endl;
                addOdometryBetweenFactor(lastPoseNum, poseNum, toPose3(lastOdometry.msg.pose), odometry, noise, graph);
                isam.update(graph, values);
            }
            lastOdometry = measurement;
            lastOdometryMeasurement = measurement;
            lastPoseNum = poseNum;
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
