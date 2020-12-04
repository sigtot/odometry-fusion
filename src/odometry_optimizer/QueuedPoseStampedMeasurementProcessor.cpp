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
        {
            lock_guard<mutex> lock(measurementMutex);
            if (measurements.size() >= minProcessCount) {
                measurement = measurements.begin()->second;
                measurements.erase(measurements.begin());
                processFn(measurement);
                lastProcessedTimestamp = measurement.msg.header.stamp.toSec();
            }
        }
        if (measurements.size() < minProcessCount) {
            cv.wait(newMeasurementNotifier);
        }
    }
}

void QueuedPoseStampedMeasurementProcessor::addMeasurement(const PoseStampedMeasurement &measurement) {
    {
        lock_guard<mutex> lock(measurementMutex);
        if (measurement.msg.header.stamp.toSec() > lastProcessedTimestamp) {
            measurements[measurement.msg.header.stamp.toSec()] = measurement;
        } else {
            cout << fixed << "Rejected measurement because its timestamp is before the one last processed ("
                 << measurement.msg.header.stamp.toSec() << " < " << lastProcessedTimestamp << ")" << endl;
        }
    }
    cv.notify_one();
}

QueuedPoseStampedMeasurementProcessor::~QueuedPoseStampedMeasurementProcessor() {
    cv.notify_all();
    processThread.join();
}

size_t QueuedPoseStampedMeasurementProcessor::size() const {
    return measurements.size();
}
