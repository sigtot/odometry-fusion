#ifndef ODOMETRY_OPTIMIZER_QUEUEDPOSESTAMPEDMEASUREMENTPROCESSOR_H
#define ODOMETRY_OPTIMIZER_QUEUEDPOSESTAMPEDMEASUREMENTPROCESSOR_H

#include <functional>
#include <mutex>
#include <condition_variable>
#include <thread>
#include "PoseStampedMeasurement.h"

using namespace std;

class QueuedPoseStampedMeasurementProcessor {
public:
    /**
     * @param processFn The function called to process the oldest measurement in the queue
     * @param minProcessCount The minimum number of items needed in the the queue for oldest item to be processed
     */
    QueuedPoseStampedMeasurementProcessor(const function<void(const PoseStampedMeasurement &)> &processFn, int minProcessCount);

    size_t size() const;

    void addMeasurement(const PoseStampedMeasurement &measurement);

    virtual ~QueuedPoseStampedMeasurementProcessor();

private:
    void waitAndProcessMessages();

    const int minProcessCount;

    double lastProcessedTimestamp = 0.0;

    mutex newMeasurementNotifierMutex;
    condition_variable cv;

    mutex measurementMutex;
    map<double, PoseStampedMeasurement> measurements;

    function<void(PoseStampedMeasurement measurement)> processFn;
    thread processThread;
};


#endif //ODOMETRY_OPTIMIZER_QUEUEDPOSESTAMPEDMEASUREMENTPROCESSOR_H
