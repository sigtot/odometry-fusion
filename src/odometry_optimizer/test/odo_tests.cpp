#include <gtest/gtest.h>
#include <iostream>
#include <ros/ros.h>
#include "../ISAMOptimizer.h"

using namespace std;

TEST(Initialization, TwoIMUBeforeRovio) {
    ros::Publisher pub; // Dummy publisher
    ISAMOptimizer isamOptimizer(&pub);
    auto noise = noiseModel::Diagonal::Sigmas(Vector6(0.02, 0.02, 0.02, 0.02, 0.02, 0.02));
    isamOptimizer.recvIMUAndUpdateState(Vector3(0, 0, 0), Vector3(0, 0, 0), ros::Time(1602855578));
    isamOptimizer.recvIMUAndUpdateState(Vector3(0.01, 0, 0), Vector3(0, 0, 0), ros::Time(1602855579));
    isamOptimizer.recvRovioOdometryAndUpdateState(Pose3(Rot3(1, 0, 0, 0), Point3(0, 0, 0)), noise);
}

TEST(Initialization, OneIMUBeforeRovio) {
    ros::Publisher pub; // Dummy publisher
    ISAMOptimizer isamOptimizer(&pub);
    auto noise = noiseModel::Diagonal::Sigmas(Vector6(0.02, 0.02, 0.02, 0.02, 0.02, 0.02));
    isamOptimizer.recvIMUAndUpdateState(Vector3(0, 0, 0), Vector3(0, 0, 0), ros::Time(1602855578));
    isamOptimizer.recvRovioOdometryAndUpdateState(Pose3(Rot3(1, 0, 0, 0), Point3(0, 0, 0)), noise);
}

TEST(Initialization, NoIMUBeforeRovio) {
    ros::Publisher pub; // Dummy publisher
    ISAMOptimizer isamOptimizer(&pub);
    auto noise = noiseModel::Diagonal::Sigmas(Vector6(0.02, 0.02, 0.02, 0.02, 0.02, 0.02));
    isamOptimizer.recvRovioOdometryAndUpdateState(Pose3(Rot3(1, 0, 0, 0), Point3(0, 0, 0)), noise);
}

TEST(EdgeCase, NoIMUBetweenRovioAndLidar) {
    ros::Publisher pub; // Dummy publisher
    ISAMOptimizer isamOptimizer(&pub);
    auto noise = noiseModel::Diagonal::Sigmas(Vector6(0.02, 0.02, 0.02, 0.02, 0.02, 0.02));

    // Initialize with two IMU measurements to ensure we have IMU values and factors
    isamOptimizer.recvIMUAndUpdateState(Vector3(0, 0, 0), Vector3(0, 0, 0), ros::Time(1602855578));
    isamOptimizer.recvIMUAndUpdateState(Vector3(0.01, 0, 0), Vector3(0, 0, 0), ros::Time(1602855579));

    // Receive rovio then lidar right after
    isamOptimizer.recvRovioOdometryAndUpdateState(Pose3(Rot3(1, 0, 0, 0), Point3(0, 0, 0)), noise);
    isamOptimizer.recvLidarOdometryAndUpdateState(Pose3(Rot3(1, 0, 0, 0), Point3(0, 0, 0)), noise);

    // Receive a new IMU measurement again that will link up with the previous IMU nodes
    isamOptimizer.recvIMUAndUpdateState(Vector3(0.01, 0, 0), Vector3(0, 0, 0), ros::Time(1602855580));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}