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
    unique_lock<std::mutex> nextProcessNotifier(notifierMutex);
    while (!ros::isShuttingDown()) {
        OdometryMeasurement measurement;
        bool haveMeasurement = false;
        {
            lock_guard<mutex> lock(measurementMutex);
            if (measurements.size() > minProcessCount) {
                measurement = measurements.begin()->second;
                measurements.erase(measurements.begin());
                haveMeasurement = true;
            }
        }
        if (haveMeasurement) {
            processFn(measurement);
        }
        cv.wait(nextProcessNotifier);
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
