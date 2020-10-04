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
    bool enabled = true;

    void echo(const T &msg);

    string publishTopic();

public:
    EchoPublisher(const string &topic, ros::NodeHandle &nh);
};

template<class T>
EchoPublisher<T>::EchoPublisher(const string &topic, ros::NodeHandle &nh) : topic(topic) {
    pub = nh.advertise<T>(publishTopic(), 1000);
    sub = nh.subscribe(topic, 1000, &EchoPublisher::echo, this);
}

template<class T>
void EchoPublisher<T>::echo(const T &msg) {
    if (enabled) {
        pub.publish(msg);
        cout << "Echoed msg on " << publishTopic() << endl;
    } else {
        cout << "This publisher is disabled and so did not echo message on " << publishTopic() << endl;
    }
}

template<class T>
string EchoPublisher<T>::publishTopic() {
    return "/echo" + topic;
}

#endif //SIMPLE_ODOMETRY_OPT_ROS_ECHOPUBLISHER_H
