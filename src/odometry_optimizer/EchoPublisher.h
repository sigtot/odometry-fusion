#ifndef SIMPLE_ODOMETRY_OPT_ROS_ECHOPUBLISHER_H
#define SIMPLE_ODOMETRY_OPT_ROS_ECHOPUBLISHER_H

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <odometry_optimizer/ToggleEcho.h>

#include <chrono>
#include <thread>

using namespace std;

template<class T>
class EchoPublisher {
private:
    string topic;
    string echoTopic;
    ros::Publisher pub;
    ros::Subscriber sub;
    bool enabled = true;
    int delayMillis = 0;

    void echo(const T &msg);

    string publishTopic();

public:
    EchoPublisher(const string &topic, ros::NodeHandle &nh, const string &echoTopic, int delayMillis = 0);
    EchoPublisher(const string &topic, ros::NodeHandle &nh) : EchoPublisher(topic, nh, topic) {};

    void enable();
    void disable();
    bool toggleEnabled(odometry_optimizer::ToggleEcho::Request &req,
                       odometry_optimizer::ToggleEcho::Response &res);
};

template<class T>
EchoPublisher<T>::EchoPublisher(const string &topic, ros::NodeHandle &nh, const string &echoTopic, int delayMillis) : topic(topic), echoTopic(echoTopic) {
    pub = nh.advertise<T>(publishTopic(), 1000);
    sub = nh.subscribe(topic, 1000, &EchoPublisher::echo, this);
    this->delayMillis = delayMillis;
}

template<class T>
void EchoPublisher<T>::echo(const T &msg) {
    if (enabled) {
        if (delayMillis > 0) {
            cout << "Delaying for " << delayMillis << " millis" << endl;
            this_thread::sleep_for(std::chrono::milliseconds(delayMillis));
        }
        pub.publish(msg);
    } else {
        cout << "This publisher is disabled so did not echo message on " << publishTopic() << endl;
    }
}

template<class T>
string EchoPublisher<T>::publishTopic() {
    return "/echo" + echoTopic;
}

template<class T>
bool EchoPublisher<T>::toggleEnabled(odometry_optimizer::ToggleEcho::Request &req,
                                     odometry_optimizer::ToggleEcho::Response &res) {
    enabled = !enabled;
    cout << "Echo on " << publishTopic() << " now " << (enabled ? "enabled" : "disabled") << endl;
    res.enabled = enabled;
    return true;
}

template<class T>
void EchoPublisher<T>::enable() {
    enabled = true;
}

template<class T>
void EchoPublisher<T>::disable() {
    enabled = false;
}

#endif //SIMPLE_ODOMETRY_OPT_ROS_ECHOPUBLISHER_H
