#ifndef ODOMETRY_OPTIMIZER_ODOMETRYMEASUREMENT_H
#define ODOMETRY_OPTIMIZER_ODOMETRYMEASUREMENT_H

#include <nav_msgs/Odometry.h>

enum OdometryType { ODOMETRY_TYPE_ROVIO, ODOMETRY_TYPE_LOAM };

struct OdometryMeasurement {
    OdometryType type;
    nav_msgs::Odometry msg;
    bool healthy;
};

#endif //ODOMETRY_OPTIMIZER_ODOMETRYMEASUREMENT_H
