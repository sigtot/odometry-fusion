#include <ros/init.h>
#include "QueuedOdometryMeasurementProcessor.h"
#include <iostream>

QueuedOdometryMeasurementProcessor::QueuedOdometryMeasurementProcessor(
        const function<void(const OdometryMeasurement &)> &processFn,
        const int minProcessCount) : processFn(processFn), minProcessCount(minProcessCount) {
    cout << "Created processor" << endl;
    processThread = thread(&QueuedOdometryMeasurementProcessor::waitAndProcessMessages, this);
}

void QueuedOdometryMeasurementProcessor::waitAndProcessMessages() {
    unique_lock<std::mutex> lock(notifierMutex);
    while (!ros::isShuttingDown()) {
        OdometryMeasurement measurement;
        bool haveMeasurement = false;
        measurementMutex.lock();
        if (measurements.size() > minProcessCount) {
            measurement = measurements.begin()->second;
            measurements.erase(measurements.begin());
            haveMeasurement = true;
        }
        measurementMutex.unlock();
        if (haveMeasurement) {
            processFn(measurement);
        }
        cv.wait(lock);
    }
}

void QueuedOdometryMeasurementProcessor::addMeasurement(const OdometryMeasurement &measurement) {
    measurementMutex.lock();
    measurements[measurement.msg.header.stamp.toSec()] = measurement;
    measurementMutex.unlock();
    cv.notify_one();
}

QueuedOdometryMeasurementProcessor::~QueuedOdometryMeasurementProcessor() {
    cv.notify_all();
    processThread.join();
}
