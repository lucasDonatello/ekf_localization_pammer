#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <eigen3/Eigen/Dense>

class Filter {
public:
    Filter() {
        // Initialize state and covariance matrix
        x_.setZero();
        P_.setIdentity();

        // Set process and measurement noise covariances
        Q_ = Eigen::Matrix3d::Identity() * 0.01;
        R_ = Eigen::Matrix2d::Identity() * 0.1;
    }

    void predict(double v, double omega, double dt) {
        // State transition model
        Eigen::Vector3d u(v * dt * cos(x_(2)), v * dt * sin(x_(2)), omega * dt);
        x_ += u;

        // Jacobian of the motion model
        Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
        F(0, 2) = -v * dt * sin(x_(2));
        F(1, 2) = v * dt * cos(x_(2));

        // Update the covariance matrix
        P_ = F * P_ * F.transpose() + Q_;
    }

    void correct(const Eigen::Vector2d& z, const Eigen::Vector2d& h, const Eigen::Matrix<double, 2, 3>& H) {
        // Compute the innovation
        Eigen::Vector2d y = z - h;

        // Compute the innovation covariance
        Eigen::Matrix2d S = H * P_ * H.transpose() + R_;

        // Compute the Kalman gain
        Eigen::Matrix<double, 3, 2> K = P_ * H.transpose() * S.inverse();

        // Update the state estimate
        x_ += K * y;

        // Update the covariance matrix
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        P_ = (I - K * H) * P_;
    }

    Eigen::Vector3d getState() const {
        return x_;
    }

    Eigen::Matrix3d getCovariance() const {
        return P_;
    }

private:
    Eigen::Vector3d x_;  // State vector [x, y, theta]
    Eigen::Matrix3d P_;  // Covariance matrix

    Eigen::Matrix3d Q_;  // Process noise covariance
    Eigen::Matrix2d R_;  // Measurement noise covariance
};

Filter ekf_filter;
ros::Publisher pose_pub;

void callback(const nav_msgs::OdometryConstPtr& odom_msg, const sensor_msgs::LaserScanConstPtr& scan_msg) {
    // Extract odometry information
    double v = odom_msg->twist.twist.linear.x;
    double omega = odom_msg->twist.twist.angular.z;
    double dt = (ros::Time::now() - odom_msg->header.stamp).toSec();

    // EKF Prediction Step
    ekf_filter.predict(v, omega, dt);

    // Find the closest range in the laser scan
    double min_range = std::numeric_limits<double>::infinity();
    int min_index = -1;
    for (int i = 0; i < scan_msg->ranges.size(); ++i) {
        if (scan_msg->ranges[i] < min_range) {
            min_range = scan_msg->ranges[i];
            min_index = i;
        }
    }

    // Calculate the measurement vector z
    Eigen::Vector2d z;
    z << min_range, scan_msg->angle_min + min_index * scan_msg->angle_increment;

    // Assuming the expected measurement h is based on the current state
    Eigen::Vector2d h;
    h << sqrt(pow(ekf_filter.getState()(0), 2) + pow(ekf_filter.getState()(1), 2)),
         atan2(ekf_filter.getState()(1), ekf_filter.getState()(0)) - ekf_filter.getState()(2);

    // Calculate the Jacobian matrix H
    Eigen::Matrix<double, 2, 3> H;
    double range = h(0);
    H << ekf_filter.getState()(0) / range, ekf_filter.getState()(1) / range, 0,
         -ekf_filter.getState()(1) / pow(range, 2), ekf_filter.getState()(0) / pow(range, 2), -1;

    // EKF Update Step
    ekf_filter.correct(z, h, H);

    // Get the updated state and covariance
    Eigen::Vector3d state = ekf_filter.getState();
    Eigen::Matrix3d covariance = ekf_filter.getCovariance();

    // Publish the pose with covariance
    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "base_link";
    pose_msg.pose.pose.position.x = state(0);
    pose_msg.pose.pose.position.y = state(1);
    pose_msg.pose.pose.orientation.z = sin(state(2) / 2.0);
    pose_msg.pose.pose.orientation.w = cos(state(2) / 2.0);

    // Fill the covariance matrix
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            if (i < 3 && j < 3) {
                pose_msg.pose.covariance[i * 6 + j] = covariance(i, j);
            } else {
                pose_msg.pose.covariance[i * 6 + j] = 0.0;
            }
        }
    }

    pose_pub.publish(pose_msg);
    ROS_INFO("Pose: x = %f, y = %f, theta = %f", state(0), state(1), state(2));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ekf_localization_node");
    ros::NodeHandle nh;

    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "odom", 1);
    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh, "scan", 1);

    pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("ekf_pose", 10);

    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::LaserScan> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub, scan_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

    return 0;
}
