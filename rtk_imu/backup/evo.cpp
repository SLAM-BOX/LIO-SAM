#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <fstream>
#include <iomanip>

using namespace std;

const std::string pose_fusion_topic_sim = "/tf/fusion_sim";
const std::string gps_debug_topic_sim = "/tf/gps_debug_sim";

std::string output_dir = "/home/xl/rtk_imu/";
std::string rtk_tum_pose_file = output_dir + "rtk_tum_pose_file.txt";
std::string fusion_tum_pose_file = output_dir + "fusion_tum_pose_file.txt";

std::ofstream rtk_tum_pose(rtk_tum_pose_file, ios::out);
std::ofstream fusion_tum_pose(fusion_tum_pose_file, ios::out);

void initFileOutput() {
    if (!rtk_tum_pose.is_open() || !fusion_tum_pose.is_open()) {
        cout << " Failed to open the output file ! " << endl;
    }

    rtk_tum_pose << "# ground truth trajectory" << std::endl;
    rtk_tum_pose << "# from: rtk-gps '" << std::endl;
    rtk_tum_pose << "# timestamp tx ty tz qx qy qz qw" << std::endl;
}

void rtkPoseCallback(const  nav_msgs::Odometry::ConstPtr &msg) {
    rtk_tum_pose << std::setprecision(20)
                    << msg->header.stamp.toSec() << " "
                    << msg->pose.pose.position.x << " "
                    << msg->pose.pose.position.y << " "
                    << msg->pose.pose.position.z << " "
                    << msg->pose.pose.orientation.x << " "
                    << msg->pose.pose.orientation.y << " "
                    << msg->pose.pose.orientation.z << " "
                    << msg->pose.pose.orientation.w << std::endl;
}

void fusionPoseCallback(const  nav_msgs::Odometry::ConstPtr &msg) {
    fusion_tum_pose << std::setprecision(20)
                    << msg->header.stamp.toSec() << " "
                    << msg->pose.pose.position.x << " "
                    << msg->pose.pose.position.y << " "
                    << msg->pose.pose.position.z << " "
                    << msg->pose.pose.orientation.x << " "
                    << msg->pose.pose.orientation.y << " "
                    << msg->pose.pose.orientation.z << " "
                    << msg->pose.pose.orientation.w << std::endl;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "evo");

    initFileOutput();

    ros::NodeHandle n;
    ros::Subscriber subRtkPose = n.subscribe<nav_msgs::Odometry>(gps_debug_topic_sim, 1000, rtkPoseCallback);
    ros::Subscriber subFusionPose = n.subscribe<nav_msgs::Odometry>(pose_fusion_topic_sim, 1000, fusionPoseCallback);
    ros::spin();

    return 0;
}