#ifndef ODOMETRY_OPTIMIZER_HEALTHBUFFER_H
#define ODOMETRY_OPTIMIZER_HEALTHBUFFER_H

#include <map>
#include <mutex>
#include <string>
#include <std_msgs/Header.h>

using namespace std;

class HealthBuffer {
private:
    const int bufferSize;
    const int degenerateThresh;

    mutex mu;
    map<double, int> buffer; // 0: healthy, 1: unhealthy

    bool degenerate = false;
    bool wasDegenerate = false;

    int unsafeBufferSum();

    /**
     * Unsafe: Must only be called from a scope with mu locked
     */
    void unsafePropagateDegenerateFSM();

public:
    HealthBuffer(int bufferSize, int degenerateThresh);

    void addMeasurement(const std_msgs::Header &measurement);

    bool isHealthy(ros::Time ts);

    bool isDegenerate() const;
    bool wasDegenerateLastTimeStep() const;

    void print(const string &prefix);
};


#endif //ODOMETRY_OPTIMIZER_HEALTHBUFFER_H
