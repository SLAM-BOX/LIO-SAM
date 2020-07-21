
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

using namespace std;

const std::string lidar_odometry_topic = "/tf/lidar_transform";
const std::string gps_position_topic = "/gps_pub";
const std::string imu_accer_angle_topic = "/imu/data";
const std::string pose_fusion_topic = "/tf/fusion";
const std::string lidar_topic = "/velodyne_points";

const std::string lidar_odometry_topic_sim = "/tf/lidar_transform";
const std::string gps_position_topic_sim = "/gps_pub_sim";
const std::string imu_accer_angle_topic_sim = "/imu/data_sim";
const std::string pose_fusion_topic_sim = "/tf/fusion";
const std::string lidar_topic_sim = "/correct_lidar";

ros::Publisher pubImu;
ros::Publisher pubGps;
ros::Publisher pubLidar;

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    sensor_msgs::Imu sim_msg = *msg;
    sim_msg.header.stamp = ros::Time::now();
    pubImu.publish(sim_msg);
}

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    sensor_msgs::NavSatFix sim_msg = *msg;
    sim_msg.header.stamp = ros::Time::now();
    pubGps.publish(sim_msg);
}

void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    sensor_msgs::PointCloud2 sim_msg = *msg;
    sim_msg.header.stamp = ros::Time::now();
    pubLidar.publish(sim_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lego_loam_sim");
    ROS_INFO("\033[1;32m---->\033[0m Convert Simlation Started.");

    ros::NodeHandle nh("~");

    pubImu = nh.advertise<sensor_msgs::Imu>(imu_accer_angle_topic_sim, 1000);
    pubGps = nh.advertise<sensor_msgs::NavSatFix>(gps_position_topic_sim, 1000);
    pubLidar = nh.advertise<sensor_msgs::PointCloud2>(lidar_topic_sim, 1000);

    ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu>(imu_accer_angle_topic, 1000, imuCallback);
    ros::Subscriber subGps = nh.subscribe<sensor_msgs::NavSatFix>(gps_position_topic, 1000, gpsCallback);
    ros::Subscriber sublidar = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 1000, lidarCallback);

    ros::spin();
    return 0;
}