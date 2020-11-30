#ifndef ODOMETRY_OPTIMIZER_IMUQUEUE_H
#define ODOMETRY_OPTIMIZER_IMUQUEUE_H

#include <map>
#include <mutex>
#include <sensor_msgs/Imu.h>
#include <gtsam/navigation/CombinedImuFactor.h>

using namespace std;
using namespace gtsam;

class IMUQueue {
private:
    map<double, sensor_msgs::Imu> imuMap;
    mutex mu;

public:
    IMUQueue();

    void addMeasurement(const sensor_msgs::Imu &measurement);

    bool hasMeasurementsInRange(ros::Time start, ros::Time end);

    void integrateIMUMeasurements(std::shared_ptr<PreintegrationType> &imuMeasurements, ros::Time start, ros::Time end);

};


#endif //ODOMETRY_OPTIMIZER_IMUQUEUE_H
