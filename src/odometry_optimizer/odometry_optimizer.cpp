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

    double covPos0, covPos1, covPos2, covVel0, covVel1, covVel2, covAtt0, covAtt1, covAtt2, covAcb0, covAcb1,
            covAcb2, covGyb0, covGyb1, covGyb2;
    cout << "Have param cov_pos_0: " << nh.getParam("cov_pos_0", covPos0) << endl;
    cout << "Have param cov_pos_1: " << nh.getParam("cov_pos_1", covPos1) << endl;
    cout << "Have param cov_pos_2: " << nh.getParam("cov_pos_2", covPos2) << endl;
    cout << "Have param cov_vel_0: " << nh.getParam("cov_vel_0", covVel0) << endl;
    cout << "Have param cov_vel_1: " << nh.getParam("cov_vel_1", covVel1) << endl;
    cout << "Have param cov_vel_2: " << nh.getParam("cov_vel_2", covVel2) << endl;
    cout << "Have param cov_att_0: " << nh.getParam("cov_att_0", covAtt0) << endl;
    cout << "Have param cov_att_1: " << nh.getParam("cov_att_1", covAtt1) << endl;
    cout << "Have param cov_att_2: " << nh.getParam("cov_att_2", covAtt2) << endl;
    cout << "Have param cov_acb_0: " << nh.getParam("cov_acb_0", covAcb0) << endl;
    cout << "Have param cov_acb_1: " << nh.getParam("cov_acb_1", covAcb1) << endl;
    cout << "Have param cov_acb_2: " << nh.getParam("cov_acb_2", covAcb2) << endl;
    cout << "Have param cov_gyb_0: " << nh.getParam("cov_gyb_0", covGyb0) << endl;
    cout << "Have param cov_gyb_1: " << nh.getParam("cov_gyb_1", covGyb1) << endl;
    cout << "Have param cov_gyb_2: " << nh.getParam("cov_gyb_2", covGyb2) << endl;

    cout << "covGyb1" << covGyb1 << endl;

    imu_params->biasAccCovariance << covAcb0, 0,       0,
                                     0,       covAcb1, 0,
                                     0,       0,       covAcb2;
    imu_params->biasOmegaCovariance << covGyb0, 0,       0,
                                       0,       covGyb1, 0,
                                       0,       0,       covGyb2;
    imu_params->accelerometerCovariance << covAtt0, 0,       0,
                                           0,       covAtt1, 0,
                                           0,       0,       covAtt2; // Is this right?
    imu_params->integrationCovariance = eye3 * 0.001; // don't know
    imu_params->gyroscopeCovariance = eye3 * 0.0035 * 3.14 / 180; // rad/s/sqrt(Hz) Wrong: From vn100
    imu_params->omegaCoriolis = Vector3::Zero(); // don't know
    auto imuBias = imuBias::ConstantBias(); // Initialize at zero bias

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
                                          &ISAMOptimizer::recvIMUMsgAndUpdateState,
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
