#include <gtest/gtest.h>
#include <iostream>
#include <ros/ros.h>
#include "../ISAMOptimizer.h"

using namespace std;

/*
 * TODO: TESTS ARE CURRENTLY NOT REALLY USED :(
 * Will hopefully find time to fix and add some actually good tests
 */

TEST(Initialization, DummyTest) {
    ros::Publisher pub; // Dummy publisher
    // Do something
}
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}