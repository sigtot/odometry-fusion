#include <ros/init.h>
#include "QueuedOdometryMeasurementProcessor.h"

QueuedOdometryMeasurementProcessor::QueuedOdometryMeasurementProcessor(
        const function<void(const OdometryMeasurement &)> &processFn,
        const int minProcessCount) : processFn(processFn), minProcessCount(minProcessCount) {
    processThread = thread(&QueuedOdometryMeasurementProcessor::waitAndProcessMessages, this);
}

void QueuedOdometryMeasurementProcessor::waitAndProcessMessages() {
    unique_lock<std::mutex> newMeasurementNotifier(newMeasurementNotifierMutex);
    while (!ros::isShuttingDown()) {
        OdometryMeasurement measurement;
        bool haveMeasurement = false;
        {
            lock_guard<mutex> lock(measurementMutex);
            if (measurements.size() >= minProcessCount) {
                measurement = measurements.begin()->second;
                measurements.erase(measurements.begin());
                haveMeasurement = true;
            }
        }
        if (haveMeasurement) {
            processFn(measurement);
        }
        if(measurements.size() < minProcessCount) {
            cv.wait(newMeasurementNotifier);
        }
    }
}

void QueuedOdometryMeasurementProcessor::addMeasurement(const OdometryMeasurement &measurement) {
    {
        lock_guard<mutex> lock(measurementMutex);
        measurements[measurement.msg.header.stamp.toSec()] = measurement;
    }
    cv.notify_one();
}

QueuedOdometryMeasurementProcessor::~QueuedOdometryMeasurementProcessor() {
    cv.notify_all();
    processThread.join();
}
