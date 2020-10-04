#ifndef SIMPLE_ODOMETRY_OPT_ROS_ECHOPUBLISHER_H
#define SIMPLE_ODOMETRY_OPT_ROS_ECHOPUBLISHER_H

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <odometry_optimizer/ToggleEcho.h>

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

    bool toggleEnabled(odometry_optimizer::ToggleEcho::Request &req,
                       odometry_optimizer::ToggleEcho::Response &res);
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
        cout << "This publisher is disabled so did not echo message on " << publishTopic() << endl;
    }
}

template<class T>
string EchoPublisher<T>::publishTopic() {
    return "/echo" + topic;
}

template<class T>
bool EchoPublisher<T>::toggleEnabled(odometry_optimizer::ToggleEcho::Request &req,
                                     odometry_optimizer::ToggleEcho::Response &res) {
    enabled = !enabled;
    cout << "Echo on " << publishTopic() << " now " << (enabled ? "enabled" : "disabled") << endl;
    res.enabled = enabled;
    return true;
}

#endif //SIMPLE_ODOMETRY_OPT_ROS_ECHOPUBLISHER_H
