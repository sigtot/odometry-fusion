#include <ros/ros.h>
#include "EchoPublisher.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "echo_node");
    ros::NodeHandle nh;
    EchoPublisher rovioEchoPublisher("/rovio/odometry", nh);
    ros::spin();
}
