#include "EchoPublisher.h"

#include <iostream>
#include <nav_msgs/Odometry.h>

EchoPublisher::EchoPublisher(const string& topic, ros::NodeHandle &nh) : topic(topic) {
    pub = nh.advertise<nav_msgs::Odometry>("/echo/" + topic, 1000);
    sub = nh.subscribe(topic, 1000, &EchoPublisher::echo, this);
}

void EchoPublisher::echo(const nav_msgs::Odometry &msg) {
    pub.publish(msg);
    cout << "Echoed msg on " << topic << endl;
}
