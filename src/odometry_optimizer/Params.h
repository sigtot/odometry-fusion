#ifndef ODOMETRY_OPTIMIZER_PARAMS_H
#define ODOMETRY_OPTIMIZER_PARAMS_H

#include "IMUQueueParams.h"

struct Params {
    bool onlyIMU = false;
    double initBiasX = 0.0;
    double initBiasY = 0.0;
    double initBiasZ = 0.0;
    double initBiasGX = 0.0;
    double initBiasGY = 0.0;
    double initBiasGZ = 0.0;
    IMUQueueParams imuQueueParams;
};

#endif //ODOMETRY_OPTIMIZER_PARAMS_H
