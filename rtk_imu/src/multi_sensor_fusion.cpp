// lego_loam
#include "utility.h"
#include <ros/ros.h>
// ros msg
//#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
// std
#include <condition_variable>
#include <queue>
#include <mutex>
#include <iostream>
#include <memory>
#include <iomanip>
// Eigen
#include <Eigen/Eigen>
#include <Eigen/Dense>
// GeographicLib
#include <GeographicLib/LocalCartesian.hpp>
// TF LIB
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace std;
using namespace Eigen;

class KalmanFilter {
public:
    /**
    * Create a Kalman filter with the specified matrices.
    *   A - System dynamics matrix
    *   C - Output matrix
    *   Q - Process noise covariance
    *   R - Measurement noise covariance
    *   P - Estimate error covariance
     *  X(k+1) = A(k_k+1) * X(k) + Q(k)
     *  Z(k) = C(k) * X(k) + R(k)
    */
    KalmanFilter(double init_t, size_t kNumStates = 15, size_t kNumMeasus = 3)
            : initialized(false), t0(init_t) {

        A = Eigen::MatrixXd(kNumStates, kNumStates); // kNumStates * kNumStates
        Q = Eigen::MatrixXd(kNumStates, kNumStates); // kNumStates * kNumStates

        C = Eigen::MatrixXd(kNumMeasus, kNumStates); // kNumMeasus  * kNumStates
        R = Eigen::MatrixXd(kNumMeasus, kNumMeasus); // kNumMeasus  * kNumMeasus

        P = Eigen::MatrixXd(kNumStates, kNumStates); // kNumStates * kNumStates
        I = Eigen::MatrixXd(kNumStates, kNumStates); // kNumStates * kNumStates

        this->A.setIdentity();
        this->C.setIdentity();
        this->C.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();
        this->x_hat = Eigen::VectorXd(kNumStates);
        this->x_hat.setZero();
        this->I.setIdentity();
        this->t = t0;

        Q.setZero();
        Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * std::pow(Sigma_imu_rpy_noise, 2);
        Q.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * std::pow(0.02, 2);
        Q.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * std::pow(0.01, 2);

        R.setIdentity();
        R *= std::pow(Sigma_fix_xyz_noise, 2);

        P.setZero();
        P.block<3,3>(0, 0) = Eigen::Matrix3d::Identity() * std::pow(Sigma_fix_xyz_noise, 2);
        P.block<3,3>(3, 3) = Eigen::Matrix3d::Identity() * std::pow(Sigma_imu_rpy_noise, 2);
        P.block<3,3>(9, 9) = Eigen::Matrix3d::Identity() * std::pow(Sigma_imu_angular_velocity_ratio_noise, 2);
        P.block<3,3>(12, 12) = Eigen::Matrix3d::Identity() * std::pow(Sigma_imu_acc_ratio_noise, 2);
        this->initialized = true;
    }

    void updateA(double dt) {
        A.setIdentity();
        A.block<3,3>(0, 6) = Eigen::Matrix3d::Identity() * dt;
        A.block<3,3>(6, 12) = Eigen::Matrix3d::Identity() * dt;
    }

    void predict(const ImuData &imu) {
        Eigen::Vector3d rpy = quaternion2euler(imu.orientation);
        radianV2degreeV(rpy);
        x_hat.segment<3>(3) = rpy;
        x_hat.segment<3>(9) = imu.angular_velocity;

        double dt = imu.timestamp - getFilterTime();

        ImuSetupParams imu_param;
        Eigen::Vector3d curr_acc = imu.orientation * (imu.linear_acceleration - imu_param.acc_bias) - imu_param.gravity;
        x_hat.segment<3>(12) =  0.5 * (curr_acc + x_hat.segment<3>(12));
        x_hat.segment<3>(6) += x_hat.segment<3>(12) * dt;
        x_hat.segment<3>(0) += x_hat.segment<3>(6) * dt;

        A.setIdentity();
        A.block<3,3>(0, 6) = Eigen::Matrix3d::Identity() * dt;
        P = A * P * A.transpose() + Q;

        // rpy covariance
        P.block<3,3>(3, 3) = Eigen::Matrix3d::Identity() * std::pow(Sigma_imu_rpy_noise, 2);
        P.block<3,3>(9, 9) = Eigen::Matrix3d::Identity() * std::pow(Sigma_imu_angular_velocity_ratio_noise, 2);
        P.block<3,3>(12, 12) = Eigen::Matrix3d::Identity() * std::pow(Sigma_imu_acc_ratio_noise, 2);

        t = imu.timestamp;
    }

    void update(double fix_t, const Eigen::Vector3d &fix) {
        // predict position and velocity in current time
        double dt = fix_t - t;
        updateA(dt);
        Eigen::VectorXd x_hat_new = predict();
        if(std::fabs(fix_t - last_fix_time) > 1e-6) {
            x_hat_new.segment<3>(6) = (fix - old_fix) / (fix_t - last_fix_time);
        }

        {
            last_fix_time = fix_t;
            old_fix = fix;
        }

        P = A * P * A.transpose() + Q;
        K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
        x_hat_new += K * (fix - C * x_hat_new);
        P = (I - K * C) * P;
        x_hat = x_hat_new;

        P.block<3,3>(0, 0) = Eigen::Matrix3d::Identity() * std::pow(Sigma_fix_xyz_noise, 2);

        t = fix_t;
    }

    Eigen::VectorXd predict() {
        return A * x_hat;
    }

    Eigen::VectorXd state() { return x_hat; };

    bool isInitialized() { return initialized; };

    Eigen::MatrixXd getP() { return P; }

    Eigen::MatrixXd getR() { return R; }

    Eigen::MatrixXd getC() { return C; }

    double getFilterTime() { return t; }

    void printKFState() {
        std::cout << "-------------" << std::endl;
        std::cout << this->state() << std::endl;
        std::cout << "-------------" << std::endl;
        std::cout << this->getP() << std::endl;
        std::cout << "-------------" << std::endl;
        std::cout << this->getR() << std::endl;
        std::cout << "-------------" << std::endl;
        std::cout << this->getC() << std::endl;
    }

private:

    // Matrices for computation
    Eigen::MatrixXd A, C, Q, R, P, K;

    // System dimensions
    int m, n;

    // Initial and current time
    double t0, t;

    // Is the filter initialized?
    bool initialized;

    // n-size identity
    Eigen::MatrixXd I;

    // Estimated states
    Eigen::VectorXd x_hat;

    double last_fix_time;
    Eigen::Vector3d old_fix;
};

class MultiSensorFusion{
private:
    ros::NodeHandle nh_;
    // subscribers and publishers
    ros::Subscriber subLaser_;
    ros::Subscriber subImu_;
    ros::Subscriber subGps_;
    ros::Publisher pubFusionPose_;

    std::condition_variable con;
    // imu queue
    const std::size_t kImuQueueSize_;
    std::queue<ImuData> imu_queue_;  // timestamp, data
    std::mutex imu_m_;
    bool has_received_imu_ = false;
    bool has_new_imu_ = false;

    // gps queue
    const std::size_t kGpsQueueSize_;
    std::queue<NavSatFixData> gps_queue_;  // timestamp, data
    std::mutex gps_m_;
    bool has_received_gps_ = false;
    // lidar odo queue
    const std::size_t kLidarOdoQueueSize_;
    std::queue<LidarOdoData> lidar_odo_queue_;  // timestamp, data
    std::mutex lidar_odo_m_;

    // timestamp and states
    std_msgs::Header currentHeader_;

    ros::Time last_imu_t_;
    ros::Time last_gps_t_;
    ros::Time current_t_;
    double init_time_;

    bool system_init_ = false;
    bool system_ready_ = false;
    Eigen::Vector3d origin_llh_;
    std::shared_ptr<GeographicLib::LocalCartesian> cart_;

    std::shared_ptr<KalmanFilter> ekf_estimator_ptr_;
    Vector3d translation_;
    Quaterniond rotation_;
    Eigen::MatrixXd covariance_;

    // debug
    bool gps_publish_ = true;
    ros::Publisher pubGpsDebugPose_;
    Quaterniond gps_debug_rotation_ = Quaterniond::Identity();
    Quaterniond nearest_orientation;

    // TF publisher
    tf::TransformBroadcaster tf_broader_;
public:

    MultiSensorFusion(size_t kImuQueueSize,size_t kGpsQueueSize, size_t kLidarOdoQueueSize, bool gps_publish = false, bool simulate = true) :
            nh_("~"),
            kImuQueueSize_(kImuQueueSize),
            kGpsQueueSize_(kGpsQueueSize),
            kLidarOdoQueueSize_(kLidarOdoQueueSize),
            gps_publish_(gps_publish) {

        if(simulate) {

            subImu_ = nh_.subscribe<sensor_msgs::Imu>(
                    imu_accer_angle_topic, 1000, &MultiSensorFusion::imuCallback, this);

            subGps_ = nh_.subscribe<sensor_msgs::NavSatFix>(
                    gps_position_topic, 1000, &MultiSensorFusion::gpsCallback, this);

            // subLaser_ = nh_.subscribe<nav_msgs::Odometry>(
            //         lidar_odometry_topic, 1000, &MultiSensorFusion::laserOdoCallback, this);

            pubFusionPose_ = nh_.advertise<nav_msgs::Odometry>(pose_fusion_topic_sim, 200);

            pubGpsDebugPose_ = nh_.advertise<nav_msgs::Odometry>(gps_debug_topic_sim, 200);

        } else {

            subImu_ = nh_.subscribe<sensor_msgs::Imu>(
                    imu_accer_angle_topic, 1000, &MultiSensorFusion::imuCallback, this);

            subGps_ = nh_.subscribe<sensor_msgs::NavSatFix>(
                    gps_position_topic, 1000, &MultiSensorFusion::gpsCallback, this);

            subLaser_ = nh_.subscribe<nav_msgs::Odometry>(
                    lidar_odometry_topic, 1000, &MultiSensorFusion::laserOdoCallback, this);

            pubFusionPose_ = nh_.advertise<nav_msgs::Odometry>(pose_fusion_topic, 200);

            pubGpsDebugPose_ = nh_.advertise<nav_msgs::Odometry>(gps_debug_topic, 200);
        }

    }

    bool systemReady() {
        if(!system_ready_) {
            system_ready_ = (has_received_gps_ && has_received_imu_);
        }
        return system_ready_;
    }

    bool systemInit() { return system_init_; }

    void initializeFusion() {

        NavSatFixData gps_front;
        ImuData imu_front;
        {
            imu_m_.lock();
            gps_m_.lock();

            while ( !gps_queue_.empty() &&
                    !imu_queue_.empty() &&
                    imu_queue_.front().timestamp > gps_queue_.front().timestamp) {
                gps_queue_.pop();
            }

            if(gps_queue_.empty() || imu_queue_.empty()) {
                return;
            } else {
                while (imu_queue_.front().timestamp < gps_queue_.front().timestamp) {
                    imu_queue_.pop();
                }
            }

            gps_front = gps_queue_.front();
            imu_front = imu_queue_.front();

            imu_m_.unlock();
            gps_m_.unlock();
        }

        ROS_INFO("\033[1;32m---->\033[ Valid IMU/GPS Data Have Been Received! Init Begins!.");

        origin_llh_ = {gps_front.latitude, gps_front.longitude, gps_front.altitude};
        cart_ = std::make_shared<GeographicLib::LocalCartesian>(origin_llh_.x(), origin_llh_.y(), origin_llh_.z());

        translation_ = Vector3d::Zero();
        rotation_ = Quaterniond::Identity();

        init_time_ = gps_front.timestamp;

        if(initKalmanFilter()) {
            current_t_ = ros::Time().fromSec(init_time_);
            last_imu_t_ = current_t_;
            last_gps_t_ = current_t_;
            system_init_ = true;

            ROS_INFO("\033[1;32m---->\033[ Multi Sensor Fusion Init Successfully! Time is: %lf", init_time_);
            ROS_INFO("\033[1;32m---->\033[ The original coordinate is: lat = %lf, lng = %lf, height = %lf", origin_llh_.x(), origin_llh_.y(), origin_llh_.z());
        }
    }

    void sensorfusion() {

        if (has_new_imu_) has_new_imu_=false;
        else return;

        // current_t_ = ros::Time::now();

        std::vector<ImuData> imus = getImuDataBetweenLast2Current(last_imu_t_, current_t_);
        if (!imus.empty()) last_imu_t_ = current_t_;

        std::vector<NavSatFixData> gpss = getGpsDataBetweenLast2Current(last_gps_t_, current_t_);
        if (!gpss.empty())  last_gps_t_ = current_t_;

        if(!imus.empty() || !gpss.empty()) {

            fusionImuAndGps(imus, gpss);
        }

        pubPose();
    }

    void inspectBuffers() {
        ROS_INFO("\033[1;32m-->\033[imu_queue size = %d", int(imu_queue_.size()));
        ROS_INFO("\033[1;32m-->\033[gps_queue size = %d", int(gps_queue_.size()));
    }

private:

    Eigen::Vector3d toLocalCartesian(const Eigen::Vector3d &blh) {
        Eigen::Vector3d local_xyz;
        cart_->Forward(blh.x(), blh.y(), blh.z(), local_xyz[0], local_xyz[1], local_xyz[2]);
        return local_xyz;
    }

    Eigen::Vector3d getImuFixXYZ(const Eigen::Vector3d &gps_fix_blh) {
        Eigen::Vector3d gps_fix_local = toLocalCartesian(gps_fix_blh);

        return euler2quaternion(ekf_estimator_ptr_->state().segment<3>(3)) * EXTRINSIC_RTK_BODY + gps_fix_local;
    }

    bool initKalmanFilter() {
        ekf_estimator_ptr_.reset(new KalmanFilter(init_time_));

        return ekf_estimator_ptr_->isInitialized();
    }

    void fusionImuAndGps(const std::vector<ImuData> &imus, const std::vector<NavSatFixData> &gpss) {
        int gps_i = 0;
        int imu_i = 0;

        for( ; gps_i<gpss.size(); gps_i++) {
            for( ; imu_i<imus.size(); imu_i++) {

                if(imus[imu_i].timestamp < gpss[gps_i].timestamp) {
                    ekf_estimator_ptr_->predict(imus[imu_i]);

                } else
                    break;
            }

            Eigen::Vector3d gps_fix_blh{gpss[gps_i].latitude, gpss[gps_i].longitude, gpss[gps_i].altitude};
            Eigen::Vector3d imu_fix = getImuFixXYZ(gps_fix_blh);
            ekf_estimator_ptr_->update(gpss[gps_i].timestamp, imu_fix);

            if(gps_publish_) pubGpsPose(imu_fix);
        }

        for( ; imu_i<imus.size(); imu_i++) {
            ekf_estimator_ptr_->predict(imus[imu_i]);
        }

        getKFEstimatorState();
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
        if (imu_queue_.size() == kImuQueueSize_) {
            imu_queue_.pop();
        }

        //* set current timestamp
        has_new_imu_ = true;
        current_t_ = msg->header.stamp;

        ImuData imu_msg;
        imu_msg.setbuf(msg);
        nearest_orientation = {msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z};
        {
            imu_m_.lock();
            imu_queue_.emplace(imu_msg);
            imu_m_.unlock();
            con.notify_one();
        }
        if( !has_received_imu_ ) has_received_imu_ = true;
    }

    std::vector<ImuData> getImuDataBetweenLast2Current(const ros::Time &last_t, const ros::Time &current_t) {
        std::vector<ImuData> imus;

        double last_timestamp = last_t.toSec();
        double curr_timestamp = current_t.toSec();
        if(last_timestamp >= curr_timestamp || imu_queue_.empty()) return imus;

        if(!imu_queue_.empty()) {
            imu_m_.lock();
            while (imu_queue_.front().timestamp < last_timestamp) {
                imu_queue_.pop();
            }
            while (imu_queue_.front().timestamp >= last_timestamp &&
                   imu_queue_.front().timestamp <= curr_timestamp) {
                imus.emplace_back(imu_queue_.front());
                imu_queue_.pop();
            }
            imu_m_.unlock();
        }

        return imus;
    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
        if (gps_queue_.size() == kGpsQueueSize_) {
            gps_queue_.pop();
        }

        NavSatFixData gps_msg;
        gps_msg.setbuf(msg);

        {
            gps_m_.lock();
            gps_queue_.emplace(gps_msg);
            gps_m_.unlock();
            con.notify_one();
        }
        if( !has_received_gps_ ) has_received_gps_ = true;
    }

    std::vector<NavSatFixData> getGpsDataBetweenLast2Current(const ros::Time &last_t, const ros::Time &current_t) {
        std::vector<NavSatFixData> gpss;

        double last_timestamp = last_t.toSec();
        double curr_timestamp = current_t.toSec();
        if(last_timestamp >= curr_timestamp || gps_queue_.empty()) return gpss;

        if(!gps_queue_.empty()) {
            gps_m_.lock();

            while (gps_queue_.front().timestamp < last_t.toSec()) {
                gps_queue_.pop();
            }

            while (gps_queue_.front().timestamp >= last_t.toSec() &&
                   gps_queue_.front().timestamp <= current_t.toSec()) {
                gpss.emplace_back(gps_queue_.front());
                gps_queue_.pop();
            }

            gps_m_.unlock();
        }

        return gpss;
    }

    void laserOdoCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        if (lidar_odo_queue_.size() == kLidarOdoQueueSize_) {
            lidar_odo_queue_.pop();
        }

        LidarOdoData lidar_odo_msg;
        lidar_odo_msg.setbuf(msg);

        {
            lidar_odo_m_.lock();
            lidar_odo_queue_.emplace(lidar_odo_msg);
            lidar_odo_m_.unlock();
            con.notify_one();
        }
    }

    std::vector<LidarOdoData> getLOdoDataBetweenLast2Current(const ros::Time &last_t, const ros::Time &current_t) {
        std::vector<LidarOdoData> lodos;
        {
            lidar_odo_m_.lock();

            while (lidar_odo_queue_.front().timestamp < last_t.toSec()) {
                lidar_odo_queue_.pop();
            }

            while (lidar_odo_queue_.front().timestamp >= last_t.toSec() &&
                   lidar_odo_queue_.front().timestamp <= current_t.toSec()) {
                lodos.emplace_back(lidar_odo_queue_.front());
                lidar_odo_queue_.pop();
            }

            lidar_odo_m_.unlock();
        }
        return lodos;
    }

    void getKFEstimatorState() {

        Eigen::VectorXd current_state = ekf_estimator_ptr_->state();

        translation_ = current_state.segment<3>(0);

        Eigen::Vector3d yrp = current_state.segment<3>(3);

        degreeV2radianV(yrp);

        rotation_ = euler2quaternion(yrp);

        covariance_ = ekf_estimator_ptr_->getP();
    }

    void pubGpsPose(const Eigen::Vector3d &fix) {
        nav_msgs::Odometry gps_debug_tf;
        gps_debug_tf.header.stamp = current_t_;
        gps_debug_tf.header.frame_id = "odom";

        Eigen::Vector3d rpy = ekf_estimator_ptr_->state().segment<3>(3);
        degreeV2radianV(rpy);
        Quaterniond rotation = euler2quaternion(rpy);

        gps_debug_tf.pose.pose.orientation.w = rotation.w();
        gps_debug_tf.pose.pose.orientation.x = rotation.x();
        gps_debug_tf.pose.pose.orientation.y = rotation.y();
        gps_debug_tf.pose.pose.orientation.z = rotation.z();

        gps_debug_tf.pose.pose.position.x = fix.x();
        gps_debug_tf.pose.pose.position.y = fix.y();
        gps_debug_tf.pose.pose.position.z = fix.z();

        // set the velocity
        gps_debug_tf.child_frame_id = "base_link";
        gps_debug_tf.twist.twist.linear.x = 0.0;
        gps_debug_tf.twist.twist.linear.y = 0.0;
        gps_debug_tf.twist.twist.angular.z = 0.0;
        pubGpsDebugPose_.publish(gps_debug_tf);
    }

    void pubPose() {
        nav_msgs::Odometry fusion_tf;
        fusion_tf.header.stamp = current_t_;
        fusion_tf.header.frame_id = "odom";
        // pose
        fusion_tf.pose.pose.orientation.x = rotation_.x();
        fusion_tf.pose.pose.orientation.y = rotation_.y();
        fusion_tf.pose.pose.orientation.z = rotation_.z();
        fusion_tf.pose.pose.orientation.w = rotation_.w();
        fusion_tf.pose.pose.position.x = translation_.x();
        fusion_tf.pose.pose.position.y = translation_.y();
        fusion_tf.pose.pose.position.z = translation_.z();

        fusion_tf.pose.covariance = {
                covariance_(0,0), covariance_(0,1), covariance_(0,2), covariance_(0,3), covariance_(0,4), covariance_(0,5),
                covariance_(1,0), covariance_(1,1), covariance_(1,2), covariance_(1,3), covariance_(1,4), covariance_(1,5),
                covariance_(2,0), covariance_(2,1), covariance_(2,2), covariance_(2,3), covariance_(2,4), covariance_(2,5),
                covariance_(3,0), covariance_(3,1), covariance_(3,2), covariance_(3,3), covariance_(3,4), covariance_(3,5),
                covariance_(4,0), covariance_(4,1), covariance_(4,2), covariance_(4,3), covariance_(4,4), covariance_(4,5),
                covariance_(5,0), covariance_(5,1), covariance_(5,2), covariance_(5,3), covariance_(5,4), covariance_(5,5)
        };

        // set the velocity
        fusion_tf.child_frame_id = "base_link";
        fusion_tf.twist.twist.linear.x = 0.0;
        fusion_tf.twist.twist.linear.y = 0.0;
        fusion_tf.twist.twist.angular.z = 0.0;

        pubFusionPose_.publish(fusion_tf);


        // publish tf
        tf::Transform tf_t;
        tf::Quaternion tf_q;
        tf_t.setRotation(tf::Quaternion(rotation_.x(), rotation_.y(), rotation_.z(), rotation_.w()));
        tf_t.setOrigin(tf::Vector3(translation_.x(), translation_.y(), translation_.z()));
        tf_broader_.sendTransform(tf::StampedTransform(tf_t, current_t_, "/odom", "/base_link"));
    }
};


int main(int argc, char** argv) {

    ros::init(argc, argv, "lego_loam");

    ROS_INFO("\033[1;32m---->\033[0m Multi Sensor Fusion ROS-NODE Started.");

    MultiSensorFusion TFusion(imu_buffer_size, gps_buffer_size, laser_buffer_size, true, true);

    ros::Rate rate(fusion_frequency);

    while (ros::ok()) {

        ros::spinOnce();

        if ( TFusion.systemReady() ) {

            if (!TFusion.systemInit()) {

                TFusion.initializeFusion();

            } else {

                TFusion.sensorfusion();

            }
        }

        rate.sleep();
    }

    return 0;
}