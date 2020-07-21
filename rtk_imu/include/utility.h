#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include <ros/ros.h>
// ros msg
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

// Eigen
#include <Eigen/Eigen>
#include <Eigen/Dense>

using namespace Eigen;

const int fusion_frequency = 20;
const int imu_buffer_size = 1000;
const int gps_buffer_size = 500;
const int laser_buffer_size = 500;

const std::string lidar_odometry_topic = "/tf/lidar_transform";
const std::string gps_position_topic = "/gps_pub";
const std::string imu_accer_angle_topic = "/imu/data";
const std::string pose_fusion_topic = "/tf/fusion";
const std::string gps_debug_topic = "/tf/gps_debug";

const std::string lidar_odometry_topic_sim = "/tf/lidar_transform_sim";
const std::string gps_position_topic_sim = "/gps_pub_sim";
const std::string imu_accer_angle_topic_sim = "/imu/data_sim";
const std::string pose_fusion_topic_sim = "/tf/fusion_sim";
const std::string gps_debug_topic_sim = "/tf/gps_debug_sim";

const double extrinsic_gps_imu_body[3] = {0.0, 0.03, 0.10};
const double Sigma_fix_xyz_noise = 0.015;
const double Sigma_imu_rpy_noise = 0.02;
const double Sigma_imu_acc_ratio_noise = 0.01;
const double Sigma_imu_angular_velocity_ratio_noise = 0.01;
const Eigen::Vector3d EXTRINSIC_RTK_BODY = {0.15, 0.002, 0.015};

struct ImuSetupParams {
    const Eigen::Vector3d gravity = {0, 0, 9.8};
    const Eigen::Vector3d acc_bias = {0.00002, 0.000002, 0};
    const Eigen::Vector3d gyr_bias = {0, 0, 0};
};

Eigen::Quaterniond euler2quaternion(const Eigen::Vector3d &euler) {
    Eigen::AngleAxisd rollAngle(AngleAxisd(euler(2),Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(AngleAxisd(euler(1),Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(AngleAxisd(euler(0),Vector3d::UnitZ()));

    return Eigen::Quaterniond(yawAngle * pitchAngle * rollAngle);
}

Eigen::Vector3d quaternion2euler(const Eigen::Quaterniond &quater) {
    return quater.matrix().eulerAngles(2,1,0);
}

double degree2radian(double degree) {
    return degree / 180.0 * M_PI;
}

void degreeV2radianV(Eigen::Vector3d &rpy) {
    for(int i=0; i<3; i++)
        rpy[i] = degree2radian(rpy[i]);
}

double radian2degree(double radian) {
    return radian / M_PI * 180.0;
}

void radianV2degreeV(Eigen::Vector3d &rpy) {
    for(int i=0; i<3; i++)
        rpy[i] = radian2degree(rpy[i]);
}

struct NavSatFixData {
    void setbuf(const sensor_msgs::NavSatFix::ConstPtr &msg) {
        timestamp = msg->header.stamp.toSec();
        latitude = msg->latitude;
        longitude = msg->longitude;
        altitude = msg->altitude;
        status = msg->status.status;
    }

    double timestamp;
    double latitude;
    double longitude;
    double altitude;
    int8_t status;
};

struct ImuData {
    void setbuf(const sensor_msgs::Imu::ConstPtr &msg) {
        timestamp = msg->header.stamp.toSec();
        orientation = {msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z};
        angular_velocity = {msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
        linear_acceleration = {msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};
    }

    double timestamp;
    Quaterniond orientation;
    Vector3d angular_velocity;
    Vector3d linear_acceleration;
};

struct LidarOdoData {
    void setbuf(const nav_msgs::Odometry::ConstPtr& msg) {
        timestamp = msg->header.stamp.toSec();
        xyz = {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
        rot = {msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z};
    }

    double timestamp;
    Eigen::Vector3d xyz;
    Eigen::Quaterniond rot;
};

#endif
