#include <ros/init.h>
#include "QueuedPoseStampedMeasurementProcessor.h"

QueuedPoseStampedMeasurementProcessor::QueuedPoseStampedMeasurementProcessor(
        const function<void(const PoseStampedMeasurement &)> &processFn,
        const int minProcessCount) : processFn(processFn), minProcessCount(minProcessCount) {
    processThread = thread(&QueuedPoseStampedMeasurementProcessor::waitAndProcessMessages, this);
}

void QueuedPoseStampedMeasurementProcessor::waitAndProcessMessages() {
    unique_lock<std::mutex> newMeasurementNotifier(newMeasurementNotifierMutex);
    while (!ros::isShuttingDown()) {
        PoseStampedMeasurement measurement;
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

void QueuedPoseStampedMeasurementProcessor::addMeasurement(const PoseStampedMeasurement &measurement) {
    {
        lock_guard<mutex> lock(measurementMutex);
        measurements[measurement.msg.header.stamp.toSec()] = measurement;
    }
    cv.notify_one();
}

QueuedPoseStampedMeasurementProcessor::~QueuedPoseStampedMeasurementProcessor() {
    cv.notify_all();
    processThread.join();
}
