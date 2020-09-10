#include "ISAMOptimizer.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>


ISAMOptimizer::ISAMOptimizer(ros::Publisher *pub, int reorderInterval) : pub(*pub),
                                                                         isam(NonlinearISAM(reorderInterval)) {
    auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
    Point3 priorPos(0, 0, 0);
    Rot3 priorRot(1, 0, 0,
                  0, 1, 0,
                  0, 0, 1);
    Pose3 priorMean(priorRot, priorPos);
    graph.add(PriorFactor<Pose3>(Symbol('x', 1), priorMean));
}

void ISAMOptimizer::recvOdometryAndPublishUpdatedPoses(const nav_msgs::Odometry &msg) {
    auto pos = msg.pose.pose.position;
    ROS_INFO("Received odometry with position: (%f, %f, %f)", pos.x, pos.y, pos.z);
    pub.publish(msg);
}
