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
#include <boost/optional/optional_io.hpp>

ISAMOptimizer::ISAMOptimizer(ros::Publisher *pub,
                             int reorderInterval) : pub(*pub),
                                                    isam(NonlinearISAM(reorderInterval)) {
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
    imuCount++;
    lastIMUTime = imuTime;
    mu.unlock();
}

void ISAMOptimizer::recvRovioOdometryAndPublishUpdatedPoses(const nav_msgs::Odometry &msg) {
    mu.lock();

    /* TODO: Composing with this makes the path look very wrong. Flipped 90 degrees. Is the transform already computed?
    Pose3 TCamIMU(Rot3(-0.4533646, -0.4579341, 0.5381071, 0.5433209),
                  Point3(0.0017640, -0.05114066, -0.0423020));
                  */
    auto odometry = toPose3(msg.pose.pose);
    if (poseNum == 0) {
        // We need to add a prior in the first iteration
        auto priorNoise = noiseModel::Diagonal::Sigmas(Vector6(0.3, 0.3, 0.3, 0.3, 0.3, 0.3));
        graph.add(PriorFactor<Pose3>(Symbol('x', 1), odometry, priorNoise)); // Init first pose on rovio odometry
        prevIMUState = NavState(odometry, Vector3()); // Init prev imu state here as well
    }

    incrementTime(msg.header.stamp);

    if (lastRovioPoseNum > 0) {
        auto odometryDelta = lastRovioOdometry.between(odometry);
        auto odometryNoise = noiseModel::Gaussian::Covariance(toGtsamMatrix(msg.pose.covariance));
        graph.add(BetweenFactor<Pose3>(Symbol('x', lastRovioPoseNum), Symbol('x', poseNum), odometryDelta, odometryNoise));
    }

    NavState imuState = imuMeasurements->predict(prevIMUState, imuBias::ConstantBias());
    Values initialEstimate;
    initialEstimate.insert(Symbol('x', poseNum), imuState.pose()); // Initialize on imu measurement.
    isam.update(graph, initialEstimate);
    prevIMUState = NavState(initialEstimate.at<Pose3>(Symbol('x', poseNum)), Vector3());
    publishNewestPose();
    resetIMUIntegrator();
    lastRovioOdometry = odometry;
    lastRovioPoseNum = poseNum;
    mu.unlock();
}

void ISAMOptimizer::recvLidarOdometryAndPublishUpdatedPoses(const nav_msgs::Odometry &msg) {
    mu.lock();
    auto odometry = toPose3(msg.pose.pose);
    if (poseNum == 0) {
        // We need to add a prior in the first iteration
        auto priorNoise = noiseModel::Diagonal::Sigmas(Vector6(0.3, 0.3, 0.3, 0.3, 0.3, 0.3));
        graph.add(PriorFactor<Pose3>(Symbol('x', 1), odometry, priorNoise)); // Init first pose on lidar odometry
        prevIMUState = NavState(odometry, Vector3()); // Init prev imu state here as well
    }
    /*
    Pose3 TLidarCam(Rot3(0.4536309, -0.4497171, -0.5423084, -0.5457794),
                    Point3(0.143373, 0.001844, -0.1527348));
    Pose3 TCamIMU(Rot3(-0.4533646, -0.4579341, 0.5381071, 0.5433209),
                  Point3(0.0017640, -0.05114066, -0.0423020));
                  */
    // TODO: we need a lastLidarPoseNum and a lastRovioPoseNum for them to connect factors between

    incrementTime(msg.header.stamp);

    if (lastLidarPoseNum > 0) {
        auto odometryDelta = lastLidarOdometry.between(odometry);
        auto odometryNoise = noiseModel::Diagonal::Sigmas(Vector6(0.2, 0.2, 0.2, 0.2, 0.2, 0.2));
        graph.add(BetweenFactor<Pose3>(Symbol('x', lastLidarPoseNum), Symbol('x', poseNum), odometryDelta,
                                       odometryNoise));
    }
    NavState imuState = imuMeasurements->predict(prevIMUState, imuBias::ConstantBias());
    Values initialEstimate;
    initialEstimate.insert(Symbol('x', poseNum), imuState.pose()); // Initialize on imu measurement.
    isam.update(graph, initialEstimate);
    prevIMUState = NavState(initialEstimate.at<Pose3>(Symbol('x', poseNum)), Vector3());
    publishNewestPose();
    resetIMUIntegrator();
    lastLidarOdometry = odometry;
    lastLidarPoseNum = poseNum;
    mu.unlock();
}

void ISAMOptimizer::publishUpdatedPoses() {
    nav_msgs::Path pathMsg;
    for (int j = 1; j < poseNum; ++j) {
        auto pose = isam.estimate().at<Pose3>(Symbol('x', j));
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
    Pose3 newPose = isam.estimate().at<Pose3>(Symbol('x', poseNum));
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
