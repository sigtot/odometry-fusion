#include "ISAMOptimizer.h"

ISAMOptimizer::ISAMOptimizer(ros::Publisher *pub) : pub(*pub) { }

void ISAMOptimizer::recvOdometryAndPublishUpdatedPoses(const nav_msgs::Odometry &msg) {
    auto pos = msg.pose.pose.position;
    ROS_INFO("Received odometry with position: (%f, %f, %f)", pos.x, pos.y, pos.z);
    pub.publish(msg);
}
