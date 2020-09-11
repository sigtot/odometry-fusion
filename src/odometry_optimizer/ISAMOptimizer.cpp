#include "ISAMOptimizer.h"
#include "utils.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

ISAMOptimizer::ISAMOptimizer(ros::Publisher *pub, int reorderInterval) : pub(*pub),
                                                                         isam(NonlinearISAM(reorderInterval)) {
    auto priorNoise = noiseModel::Diagonal::Sigmas(Vector6(0.3, 0.3, 0.3, 0.3, 0.3, 0.3));
    graph.add(PriorFactor<Pose3>(Symbol('x', poseNum), Pose3(), priorNoise));
    Values initialEstimate;
    initialEstimate.insert(Symbol('x', poseNum), Pose3());
    isam.update(graph, initialEstimate);
    poseNum++;
}

void ISAMOptimizer::recvOdometryAndPublishUpdatedPoses(const nav_msgs::Odometry &msg) {
    auto odometry = toPose3(msg.pose.pose);
    auto lastPose = isam.estimate().at<Pose3>(Symbol('x', poseNum - 1));
    auto odometryDelta = lastPose.between(odometry);
    auto odometryNoise = noiseModel::Diagonal::Sigmas(Vector6(0.2, 0.2, 0.2, 0.2, 0.2, 0.2));
    graph.add(BetweenFactor<Pose3>(Symbol('x', poseNum - 1), Symbol('x', poseNum), odometryDelta, odometryNoise));

    Values initialEstimate;
    initialEstimate.insert(Symbol('x', poseNum), odometry);
    isam.update(graph, initialEstimate);
    Pose3 newPose = isam.estimate().at<Pose3>(Symbol('x', poseNum));

    pub.publish(toPoseMsg(newPose));
    ROS_INFO("Published new pose: (%f, %f, %f)", newPose.x(), newPose.y(), newPose.z());
    poseNum++;
}
