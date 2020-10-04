#include <ros/ros.h>
#include <iostream>

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "echo_node");
    ros::NodeHandle nh;
    cout << "Hello from echo node" << endl;
    return 1;
}
