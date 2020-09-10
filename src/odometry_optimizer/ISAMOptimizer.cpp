#include "ISAMOptimizer.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>

ISAMOptimizer::ISAMOptimizer(ros::Publisher *pub, int reorderInterval) : pub(*pub),
                                                                         isam(NonlinearISAM(reorderInterval)) {
    auto priorNoise = noiseModel::Diagonal::Sigmas(Vector6(0.3, 0.3, 0.3, 0.3, 0.3, 0.3));
    graph.add(PriorFactor<Pose3>(Symbol('x', poseNum), Pose3(), priorNoise));
    Values initialEstimate;
    initialEstimate.insert(Symbol('x', poseNum), Pose3());
    isam.update(graph, initialEstimate);
    poseNum++;
}

geometry_msgs::Pose toPoseMsg(const Pose3 &pose) {
    geometry_msgs::Point pointMsg;
    pointMsg.x = pose.x();
    pointMsg.y = pose.y();
    pointMsg.z = pose.z();

    geometry_msgs::Quaternion quatMsg;
    quatMsg.w = pose.rotation().quaternion().w();
    quatMsg.x = pose.rotation().quaternion().x();
    quatMsg.y = pose.rotation().quaternion().y();
    quatMsg.z = pose.rotation().quaternion().z();

    geometry_msgs::Pose poseMsg;
    poseMsg.position = pointMsg;
    poseMsg.orientation = quatMsg;
    return poseMsg;
}

Pose3 toPose(const geometry_msgs::Pose &poseMsg) {
    auto pointMsg = poseMsg.position;
    auto point = Point3(pointMsg.x, pointMsg.y, pointMsg.z);
    auto quatMsg = poseMsg.orientation;
    auto rot = Rot3::Quaternion(quatMsg.w, quatMsg.x, quatMsg.y, quatMsg.z);
    return Pose3(rot, point);
}

void ISAMOptimizer::recvOdometryAndPublishUpdatedPoses(const nav_msgs::Odometry &msg) {
    Values est = isam.estimate();
    Pose3 lastPose = isam.estimate().at<Pose3>(Symbol('x', poseNum - 1));
    auto odometry = toPose(msg.pose.pose);
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
