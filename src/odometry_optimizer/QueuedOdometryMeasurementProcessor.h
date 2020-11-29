#ifndef ODOMETRY_OPTIMIZER_QUEUEDODOMETRYMEASUREMENTPROCESSOR_H
#define ODOMETRY_OPTIMIZER_QUEUEDODOMETRYMEASUREMENTPROCESSOR_H

#include <functional>
#include <mutex>
#include <condition_variable>
#include <thread>
#include "OdometryMeasurement.h"

using namespace std;

class QueuedOdometryMeasurementProcessor {
public:
    /**
     * @param processFn The function called to process the oldest measurement in the queue
     * @param minProcessCount The minimum number of items needed in the the queue for oldest item to be processed
     */
    QueuedOdometryMeasurementProcessor(const function<void(const OdometryMeasurement &)> &processFn, int minProcessCount);

    void addMeasurement(const OdometryMeasurement &measurement);

    virtual ~QueuedOdometryMeasurementProcessor();

private:
    void waitAndProcessMessages();

    const int minProcessCount;

    mutex notifierMutex;
    condition_variable cv;

    mutex measurementMutex;
    map<double, OdometryMeasurement> measurements;

    function<void(OdometryMeasurement measurement)> processFn;
    thread processThread;
};


#endif //ODOMETRY_OPTIMIZER_QUEUEDODOMETRYMEASUREMENTPROCESSOR_H
