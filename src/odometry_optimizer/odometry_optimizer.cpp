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


    bool haveNoiseParams = true;
    double gyroNoiseDensity, gyroRandomWalk, accNoiseDensity, accRandomWalk, rovioCovariance, loamCovariance, gravitationalAcceleration, integrationCovariance, biasAccOmegaInt;
    haveNoiseParams = haveNoiseParams && nh.getParam("gyro_noise_density", gyroNoiseDensity);
    haveNoiseParams = haveNoiseParams && nh.getParam("gyro_random_walk", gyroRandomWalk);
    haveNoiseParams = haveNoiseParams && nh.getParam("acc_noise_density", accNoiseDensity);
    haveNoiseParams = haveNoiseParams && nh.getParam("acc_random_walk", accRandomWalk);
    haveNoiseParams = haveNoiseParams && nh.getParam("integration_covariance", integrationCovariance);
    haveNoiseParams = haveNoiseParams && nh.getParam("bias_acc_omega_int", biasAccOmegaInt);
    haveNoiseParams = haveNoiseParams && nh.getParam("rovio_covariance", rovioCovariance);
    haveNoiseParams = haveNoiseParams && nh.getParam("loam_covariance", loamCovariance);
    haveNoiseParams = haveNoiseParams && nh.getParam("gravitational_acceleration", gravitationalAcceleration);

    if (!haveNoiseParams) {
        cout << "You must supply noise params (imu, rovio and loam). Exiting." << endl;
        return 1;
    }

    auto imu_params = PreintegratedCombinedMeasurements::Params::MakeSharedU(gravitationalAcceleration);

    imu_params->gyroscopeCovariance << I_3x3 * gyroNoiseDensity * gyroNoiseDensity;
    imu_params->biasOmegaCovariance << I_3x3 * gyroRandomWalk * gyroRandomWalk;
    imu_params->accelerometerCovariance << I_3x3 * accNoiseDensity * accNoiseDensity;
    imu_params->biasAccCovariance << I_3x3 * accRandomWalk * accRandomWalk;
    imu_params->integrationCovariance = I_3x3 * integrationCovariance;

    auto pub = nh.advertise<nav_msgs::Odometry>("/optimized_pose", 1000);

    ISAMOptimizer isamOptimizer(&pub, imu_params, rovioCovariance, loamCovariance);
    std::string imuTopicName, odometry1TopicName, odometry2TopicName, loamHealthTopicName;

    if (!nh.getParam("imu_topic_name", imuTopicName)) {
        cout << "Please supply a value for imu_topic_name" << endl;
    }
    nh.getParam("odometry1_topic_name", odometry1TopicName);
    nh.getParam("odometry2_topic_name", odometry2TopicName);
    nh.getParam("loam_health_topic_name", loamHealthTopicName);

    ros::Subscriber subIMU = nh.subscribe(imuTopicName,
                                          1000,
                                          &ISAMOptimizer::recvIMUMsg,
                                          &isamOptimizer);
    ros::Subscriber subRovio = nh.subscribe(odometry1TopicName,
                                            1000,
                                            &ISAMOptimizer::recvRovioOdometryAndAddToQueue,
                                            &isamOptimizer);
    ros::Subscriber subLidar = nh.subscribe(odometry2TopicName,
                                            1000,
                                            &ISAMOptimizer::recvLidarOdometryAndAddToQueue,
                                            &isamOptimizer);
    ros::Subscriber subLoamHealth = nh.subscribe(loamHealthTopicName,
                                                 1000,
                                                 &ISAMOptimizer::recvLoamHealthMsg,
                                                 &isamOptimizer);
    ros::spin();
}
