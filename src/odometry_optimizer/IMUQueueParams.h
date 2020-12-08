#ifndef ODOMETRY_OPTIMIZER_IMUQUEUEPARAMS_H
#define ODOMETRY_OPTIMIZER_IMUQUEUEPARAMS_H

struct IMUQueueParams {
    double imuTimeOffset = 0.0; ///< Time offset to add to IMU msg stamp
};

#endif //ODOMETRY_OPTIMIZER_IMUQUEUEPARAMS_H
