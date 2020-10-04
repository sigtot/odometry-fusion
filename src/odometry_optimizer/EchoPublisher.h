#ifndef SIMPLE_ODOMETRY_OPT_ROS_ECHOPUBLISHER_H
#define SIMPLE_ODOMETRY_OPT_ROS_ECHOPUBLISHER_H

#include <ros/ros.h>
#include <iostream>
#include <string>

using namespace std;

template<class T>
class EchoPublisher {
private:
    string topic;
    ros::Publisher pub;
    ros::Subscriber sub;

    void echo(const T &msg);

public:
    EchoPublisher(const string &topic, ros::NodeHandle &nh);
};

template<class T>
EchoPublisher<T>::EchoPublisher(const string &topic, ros::NodeHandle &nh) : topic(topic) {
    pub = nh.advertise<T>("/echo/" + topic, 1000);
    sub = nh.subscribe(topic, 1000, &EchoPublisher::echo, this);
}

template<class T>
void EchoPublisher<T>::echo(const T &msg) {
    pub.publish(msg);
    cout << "Echoed msg on " << topic << endl;
}

#endif //SIMPLE_ODOMETRY_OPT_ROS_ECHOPUBLISHER_H
