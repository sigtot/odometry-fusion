#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "ISAMOptimizer.h"
#include <iostream>
#include <gtsam/navigation/CombinedImuFactor.h>

using namespace std;
using namespace gtsam;

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometry_optimizer");
    ros::NodeHandle nh;

    auto imu_params = PreintegratedCombinedMeasurements::Params::MakeSharedU(9.81);
    Matrix3 eye3;
    eye3 << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;

    bool have_imu_params = true;
    double gyroNoiseDensity, gyroRandomWalk, accNoiseDensity, accRandomWalk;
    have_imu_params = have_imu_params && nh.getParam("gyro_noise_density", gyroNoiseDensity);
    have_imu_params = have_imu_params && nh.getParam("gyro_random_walk", gyroRandomWalk);
    have_imu_params = have_imu_params && nh.getParam("acc_noise_density", accNoiseDensity);
    have_imu_params = have_imu_params && nh.getParam("acc_random_walk", accRandomWalk);

    if (!have_imu_params) {
        cout << "You must supply imu params. Exiting." << endl;
        return 1;
    }

    imu_params->gyroscopeCovariance << I_3x3 * gyroNoiseDensity * gyroNoiseDensity;
    imu_params->biasOmegaCovariance << I_3x3 * gyroRandomWalk * gyroRandomWalk;
    imu_params->accelerometerCovariance << I_3x3 * accNoiseDensity * accNoiseDensity;
    imu_params->biasAccCovariance << I_3x3 * accRandomWalk * accRandomWalk;

    imu_params->integrationCovariance = I_3x3 * 1e-8; // From gtsam kitti example

    imu_params->omegaCoriolis = Vector3::Zero(); // don't know
    imu_params->biasAccOmegaInt << I_6x6 * 1e-5; // From gtsam kitti example

    auto pub = nh.advertise<nav_msgs::Odometry>("/optimized_pose", 1000);

    ISAMOptimizer isamOptimizer(&pub, imu_params);
    std::string imuTopicName, odometry1TopicName, odometry2TopicName;

    if (!nh.getParam("imu_topic_name", imuTopicName)) {
        cout << "Please supply a value for imu_topic_name" << endl;
    }
    nh.getParam("odometry1_topic_name", odometry1TopicName);
    nh.getParam("odometry2_topic_name", odometry2TopicName);

    ros::Subscriber subIMU = nh.subscribe(imuTopicName,
                                          1000,
                                          &ISAMOptimizer::safeAddIMUMsgToDeque,
                                          &isamOptimizer);
    ros::Subscriber subRovio = nh.subscribe(odometry1TopicName,
                                            1000,
                                            &ISAMOptimizer::recvRovioOdometryMsgAndPublishUpdatedPoses,
                                            &isamOptimizer);
    ros::Subscriber subLidar = nh.subscribe(odometry2TopicName,
                                            1000,
                                            &ISAMOptimizer::recvLidarOdometryMsgAndPublishUpdatedPoses,
                                            &isamOptimizer);
    ros::spin();
}
