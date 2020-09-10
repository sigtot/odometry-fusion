#include <iostream>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

using namespace std;
using namespace gtsam;

void odometryCallback(const nav_msgs::Odometry &msg){
    auto pos = msg.pose.pose.position;
    ROS_INFO("I heard: (%f, %f, %f)", pos.x, pos.y, pos.z);
}

int main(int argc, char **argv) {
    // Create an empty nonlinear factor graph
    NonlinearFactorGraph graph;

    // Add a prior on the first pose, setting it to the origin
    // A prior factor consists of a mean and a noise model (covariance matrix)
    Pose2 priorMean(0.0, 0.0, 0.0);  // prior at origin
    auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
    graph.add(PriorFactor<Pose2>(1, priorMean, priorNoise));

    // Add odometry factors
    Pose2 odometry(2.0, 0.0, 0.0);
    // For simplicity, we will use the same noise model for each odometry factor
    auto odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
    // Create odometry (Between) factors between consecutive poses
    graph.add(BetweenFactor<Pose2>(1, 2, odometry, odometryNoise));
    graph.add(BetweenFactor<Pose2>(2, 3, odometry, odometryNoise));
    graph.print("\nFactor Graph:\n");  // print

    // Create the data structure to hold the initialEstimate estimate to the solution
    // For illustrative purposes, these have been deliberately set to incorrect values
    Values initial;
    initial.insert(1, Pose2(0.5, 0.0, 0.2));
    initial.insert(2, Pose2(2.3, 0.1, -0.2));
    initial.insert(3, Pose2(4.1, 0.1, 0.1));
    initial.print("\nInitial Estimate:\n");  // print

    // optimize using Levenberg-Marquardt optimization
    Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
    result.print("Final Result:\n");

    // Calculate and print marginal covariances for all variables
    cout.precision(2);
    Marginals marginals(graph, result);
    cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
    cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
    cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;
    // You should see the covariances ballooning!

    ros::init(argc, argv, "odometry_optimizer");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/rovio/odometry", 1000, odometryCallback);
    ros::spin();
}
