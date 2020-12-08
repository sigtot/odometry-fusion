#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <iostream>
#include <gtsam/navigation/CombinedImuFactor.h>
#include "ISAMOptimizer.h"
#include "PointCloudPublisher.h"
#include "Params.h"

using namespace std;
using namespace gtsam;

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometry_optimizer");
    ros::NodeHandle nh;

    tf::TransformListener tfListener;
    cout << "Looking up camera_init transform..." << endl;
    tf::StampedTransform cameraInitTransform;
    while (!ros::isShuttingDown()) {
        // Can't make waitForTransform work for some reason, so doing this ad-hoc solution instead
        try {
            tfListener.lookupTransform("/map", "/camera_init_CORRECTED", ros::Time(0), cameraInitTransform);
            break;
        } catch (tf::LookupException &e) {
            cout << "Still waiting for transform from map to camera_init..." << endl;
            this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }
    cout << "Have transform for camera_init: " << cameraInitTransform.stamp_ << endl;

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

    int extraRovioPriorInterval;
    if (!nh.getParam("extra_rovio_prior_interval", extraRovioPriorInterval)) {
        cout << "You must supply a value for extra_rovio_prior_interval (give 0 for no extra priors)" << endl;
        return 1;
    }

    auto pub = nh.advertise<nav_msgs::Odometry>("/optimized_pose", 1000);
    auto pathPublisher = nh.advertise<nav_msgs::Path>("/optimized_path", 1000);

    Params params;
    nh.getParam("only_imu", params.onlyIMU);
    nh.getParam("init_bias_x", params.initBiasX);
    nh.getParam("init_bias_y", params.initBiasY);
    nh.getParam("init_bias_z", params.initBiasZ);
    nh.getParam("init_bias_g_x", params.initBiasGX);
    nh.getParam("init_bias_g_y", params.initBiasGY);
    nh.getParam("init_bias_g_z", params.initBiasGZ);

    nh.getParam("imu_time_offset", params.imuQueueParams.imuTimeOffset);

    ISAMOptimizer isamOptimizer(&pub, &pathPublisher, imu_params, tf::StampedTransform(), rovioCovariance,
                                loamCovariance,
                                extraRovioPriorInterval, params);
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

    PointCloudPublisherParams pointCloudPublisherParams;
    pointCloudPublisherParams.interval = 5;
    pointCloudPublisherParams.lidarFrameId = "/aft_mapped_to_init_CORRECTED";
    pointCloudPublisherParams.lidarInitFrameId = "/camera_init";
    pointCloudPublisherParams.liveFrameId = "/velodyne_fused";
    pointCloudPublisherParams.finalFrameId = "/map";
    pointCloudPublisherParams.finalPublishTime = 1561411140.726387;
    bool publishPointCloud = false;
    ros::Publisher livePointCloudPub;
    ros::Publisher finalPointCloudPub;
    ros::Subscriber subPointCloud;
    ros::Subscriber subOptimizedPath;
    PointCloudPublisher pointCloudPublisher(livePointCloudPub, finalPointCloudPub, pointCloudPublisherParams);
    if (nh.getParam("publish_point_cloud", publishPointCloud) && publishPointCloud) {
        livePointCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_in_fused_frame", 10);
        finalPointCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_final_fused_corrected", 10);
        subPointCloud = nh.subscribe("/velodyne_cloud_registered",
                                     10,
                                     &PointCloudPublisher::storeAndRepublishInNewFrame,
                                     &pointCloudPublisher);
        subOptimizedPath = nh.subscribe("/optimized_path",
                                     1000,
                                     &PointCloudPublisher::recvNewPath,
                                     &pointCloudPublisher);
    }

    ros::spin();
}
