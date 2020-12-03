#include <cmath>
#include "HealthBuffer.h"

#define SAME_TIME_THRESH 0.1

HealthBuffer::HealthBuffer(int bufferSize, int degenerateThresh) : bufferSize(bufferSize),
                                                                   degenerateThresh(degenerateThresh) {}

void HealthBuffer::addMeasurement(const std_msgs::Header &measurement) {
    lock_guard<mutex> lock(mu);
    if (buffer.size() >= bufferSize) {
        buffer.erase(buffer.begin());
    }
    buffer[measurement.stamp.toSec()] = measurement.seq;
    unsafePropagateDegenerateFSM();
}

bool HealthBuffer::isHealthy(ros::Time ts) {
    lock_guard<mutex> lock(mu);
    if (degenerate) {
        return false;
    }
    pair<double, int> closest = *buffer.begin();
    double closestTimeDiff = abs(closest.first - ts.toSec());
    for (auto &it : buffer) {
        double timeDiff = abs(it.first - ts.toSec());
        if (timeDiff < closestTimeDiff) {
            closest = it;
            closestTimeDiff = timeDiff;
        }
    }
    if (closestTimeDiff < SAME_TIME_THRESH) {
        return closest.second == 0;
    } else {
        cout << "Found no health msg with time diff less than " << SAME_TIME_THRESH << " closest was "
             << closestTimeDiff << ". measurement: " << ts.toSec() << " health msg: " << closest.first << endl;
        return false;
    }
}

bool HealthBuffer::isDegenerate() const {
    return degenerate;
}

bool HealthBuffer::wasDegenerateLastTimeStep() const {
    return wasDegenerate;
}

void HealthBuffer::print(const string &prefix) {
    lock_guard<mutex> lock(mu);
    cout << prefix;

    cout << "timestamps: " << "[";
    for (auto &it : buffer) {
        cout << it.first;
        cout << ",";
    }
    cout << "]" << endl;

    cout << "health val: " << "[";
    for (auto &it : buffer) {
        cout << it.second;
        cout << ",";
    }
    cout << "]" << endl;

    cout << "degenerate: " << degenerate << endl;
}

int HealthBuffer::unsafeBufferSum() {
    int sum = 0;
    for (auto &it : buffer) {
        sum += it.second;
    }
    return sum;
}

void HealthBuffer::unsafePropagateDegenerateFSM() {
    wasDegenerate = degenerate;
    if (degenerate) {
        if (unsafeBufferSum() == 0) {
            degenerate = false;
            cout << "No longer degenerate!" << endl;
        }
    } else {
        if (unsafeBufferSum() >= degenerateThresh) {
            cout << "Degenerate!" << endl;
            degenerate = true;
        }
    }
}
