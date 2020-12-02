#include "IMUQueue.h"
#include <iostream>

IMUQueue::IMUQueue() = default;

void IMUQueue::addMeasurement(const sensor_msgs::Imu &measurement) {
    lock_guard<mutex> lock(mu);
    imuMap[measurement.header.stamp.toSec()] = measurement;
}

bool IMUQueue::hasMeasurementsInRange(ros::Time start, ros::Time end) {
    lock_guard<mutex> lock(mu);
    int betweenCount = 0;
    for (auto &it : imuMap) {
        auto imuStamp = it.second.header.stamp;
        if (imuStamp > end) {
            break;
        }
        if (imuStamp > start) {
            ++betweenCount;
        }
    }
    return betweenCount > 1; // TODO make it > 0?
}

int IMUQueue::integrateIMUMeasurements(std::shared_ptr<PreintegrationType> &imuMeasurements, ros::Time start, ros::Time end) {
    lock_guard<mutex> lock(mu);
    int numIntg = 0;
    auto lastTime = start;
    bool ranToEnd = true;
    for (auto &it : imuMap) {
        auto imuMsg = it.second;
        if (imuMsg.header.stamp > end) {
            ranToEnd = false;
            break;
        }
        if (imuMsg.header.stamp > start) {
            auto dt = imuMsg.header.stamp - lastTime;
            auto linearMsg = imuMsg.linear_acceleration;
            auto acc = Vector3(linearMsg.x, linearMsg.y, linearMsg.z);
            auto angularMsg = imuMsg.angular_velocity;
            auto omega = Vector3(angularMsg.x, angularMsg.y, angularMsg.z);
            imuMeasurements->integrateMeasurement(acc, omega, dt.toSec());
            lastTime = imuMsg.header.stamp;
            numIntg++;
        }
    }
    if (ranToEnd) {
        cout << "WARN: imu integration ran to the end of the queue. This could imply that some msgs were lost" << endl;
    }
    return numIntg;
}
