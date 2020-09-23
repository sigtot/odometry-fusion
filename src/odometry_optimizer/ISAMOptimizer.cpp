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

ISAMOptimizer::ISAMOptimizer(ros::Publisher *pub,
                             int reorderInterval) : pub(*pub),
                                                    isam(NonlinearISAM(reorderInterval)) {}

void ISAMOptimizer::recvIMUOdometryAndPublishUpdatedPoses(const nav_msgs::Odometry &msg) {
    mu.lock();
    incrementTime(msg.header.stamp);
    auto odometry = toPose3(msg.pose.pose);
    if (poseNum > 1) {
        auto odometryDelta = lastIMUOdometry.between(odometry);
        auto odometryNoise = noiseModel::Diagonal::Sigmas(Vector6(0.4, 0.4, 0.4, 0.4, 0.4, 0.4));
        graph.add(BetweenFactor<Pose3>(Symbol('x', poseNum - 1), Symbol('x', poseNum), odometryDelta, odometryNoise));
    } else {
        // We need to add a prior in the first iteration
        auto priorNoise = noiseModel::Diagonal::Sigmas(Vector6(0.3, 0.3, 0.3, 0.3, 0.3, 0.3));
        graph.add(PriorFactor<Pose3>(Symbol('x', 1), odometry, priorNoise));
    }

    Values initialEstimate;
    initialEstimate.insert(Symbol('x', poseNum), odometry);
    isam.update(graph, initialEstimate);
    publishUpdatedPoses();
    lastIMUOdometry = odometry;
    mu.unlock();
}

void ISAMOptimizer::recvLidarOdometryAndPublishUpdatedPoses(const nav_msgs::Odometry &msg) {
    Pose3 TLidarCam(Rot3(0.4536309, -0.4497171, -0.5423084, -0.5457794),
                    Point3(0.143373, 0.001844, -0.1527348));
    Pose3 TCamIMU(Rot3(-0.4533646, -0.4579341, 0.5381071, 0.5433209),
                  Point3(0.0017640, -0.05114066, -0.0423020));
    mu.lock();
    auto odometry = TCamIMU.compose(TLidarCam).compose(toPose3(msg.pose.pose));
    if (lastLidarPoseNum > 1) {
        auto odometryDelta = lastLidarOdometry.between(odometry);
        auto odometryNoise = noiseModel::Diagonal::Sigmas(Vector6(0.2, 0.2, 0.2, 0.2, 0.2, 0.2));
        graph.add(BetweenFactor<Pose3>(Symbol('x', lastLidarPoseNum), Symbol('x', poseNum), odometryDelta,
                                       odometryNoise));

        Values initialEstimate;
        isam.update(graph, initialEstimate); // Naive implementation: Do not add a new value
        publishUpdatedPoses();
    }
    lastLidarOdometry = odometry;
    lastLidarPoseNum = poseNum;
    mu.unlock();
}

void ISAMOptimizer::publishUpdatedPoses() {
    Pose3 newPose = isam.estimate().at<Pose3>(Symbol('x', poseNum));
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

void ISAMOptimizer::incrementTime(const ros::Time &stamp) {
    poseNum++;
    timestamps.push_back(stamp);
}
