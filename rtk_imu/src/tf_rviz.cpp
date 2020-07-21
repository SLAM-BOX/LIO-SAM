#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
const std::string tf_pose_topic = "/pose_tf_debug";
const std::string imu_accer_angle_topic = "/imu/data";

ros::Publisher pubPose;
ros::Publisher pubGps;

double x = 0.0;
double y = 0.0;
double z = 0.0;
const Eigen::Vector3d acc_bias = {-0.02, -0.02, 0};
const Eigen::Vector3d gravity = {0, 0, -9.8};


bool init = false;
Eigen::Vector3d velocity, curr_acc, position;
double last_t = 0, dt = 0;

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    if(!init) {
        last_t = msg->header.stamp.toSec();
        init = true;
        return;
    }

    nav_msgs::Odometry imu_tf;
    imu_tf.header.stamp = msg->header.stamp;
    imu_tf.header.frame_id = "odom";

    imu_tf.pose.pose.orientation.x = msg->orientation.x;
    imu_tf.pose.pose.orientation.y = msg->orientation.y;
    imu_tf.pose.pose.orientation.z = msg->orientation.z;
    imu_tf.pose.pose.orientation.w = msg->orientation.w;

    dt = msg->header.stamp.toSec() - last_t;
    Eigen::Quaterniond q{msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z};
    Eigen::Vector3d linear_acceleration = {msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};
    curr_acc = 0.5 * (q * (linear_acceleration - acc_bias) + gravity + curr_acc);
    velocity += curr_acc * dt;
    position += velocity * dt;

    imu_tf.pose.pose.position.x = position.x();
    imu_tf.pose.pose.position.y = position.y();
    imu_tf.pose.pose.position.z = position.z();

    //set the velocity
    imu_tf.child_frame_id = "base_link";
    imu_tf.twist.twist.linear.x = 0.0;
    imu_tf.twist.twist.linear.y = 0.0;
    imu_tf.twist.twist.angular.z = 0.0;

    pubPose.publish(imu_tf);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "odometry");

    ros::NodeHandle n;

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

    pubPose = n.advertise<nav_msgs::Odometry>(tf_pose_topic, 5);

    ros::Subscriber subImu = n.subscribe<sensor_msgs::Imu>(imu_accer_angle_topic, 1000, imuCallback);

    ros::Rate r(20.0);

    while(n.ok()){
        ros::spinOnce();
        r.sleep();
    }
}