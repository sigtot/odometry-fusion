#include <gtest/gtest.h>
#include "../ISAMOptimizer.h"
#include <iostream>
#include <ros/ros.h>

TEST(ItWorks, itWorksCase1) {
    cout << "hello" << endl;
    EXPECT_EQ(1, 1);
    /*
    ros::Publisher pub; // Dummy publisher
    ISAMOptimizer isamOptimizer(&pub);
    auto noise = noiseModel::Diagonal::Sigmas(Vector6(0.02, 0.02, 0.02, 0.02, 0.02, 0.02));
    isamOptimizer.recvLidarOdometryAndUpdateState(Pose3(Rot3(1, 0, 0, 0), Point3(0, 0, 0)), noise);
     */
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}