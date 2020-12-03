#ifndef ODOMETRY_OPTIMIZER_POSESTAMPEDMEASUREMENT_H
#define ODOMETRY_OPTIMIZER_POSESTAMPEDMEASUREMENT_H

#include <geometry_msgs/PoseStamped.h>

enum OdometryType { ODOMETRY_TYPE_ROVIO, ODOMETRY_TYPE_LOAM };

struct PoseStampedMeasurement {
    OdometryType type;
    geometry_msgs::PoseStamped msg;
};

#endif //ODOMETRY_OPTIMIZER_POSESTAMPEDMEASUREMENT_H
