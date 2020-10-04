#include <ros/ros.h>
#include "EchoPublisher.h"

#include <nav_msgs/Odometry.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "echo_node");
    ros::NodeHandle nh;
    EchoPublisher<nav_msgs::Odometry> rovioEchoPublisher("/rovio/odometry", nh);
    EchoPublisher<nav_msgs::Odometry> lidarEchoPublisher("/aft_mapped_to_init_CORRECTED", nh);
    ros::spin();
}
