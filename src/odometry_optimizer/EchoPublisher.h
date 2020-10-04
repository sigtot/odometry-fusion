#ifndef SIMPLE_ODOMETRY_OPT_ROS_ECHOPUBLISHER_H
#define SIMPLE_ODOMETRY_OPT_ROS_ECHOPUBLISHER_H

#include <ros/ros.h>
#include <string>
#include <nav_msgs/Odometry.h>

using namespace std;

class EchoPublisher {
private:
    string topic;
    ros::Publisher pub;
    ros::Subscriber sub;

    void echo(const nav_msgs::Odometry &msg);

public:
    EchoPublisher(const string& topic, ros::NodeHandle &nh);
};


#endif //SIMPLE_ODOMETRY_OPT_ROS_ECHOPUBLISHER_H
