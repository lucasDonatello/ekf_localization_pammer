#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace std;

class Filter
{
public:
    Filter(ros::NodeHandle &nh_, ros::NodeHandle &pub_) : nh(nh_), pub(pub_), odom_sub(nh, "odom", 1), scan_sub(nh, "scan", 1), sync(MySyncPolicy(10), odom_sub, scan_sub)
    {
        nh.getParam("/ekf_node/fx", feature(0));
        nh.getParam("/ekf_node/fy", feature(1));

        nh.getParam("/ekf_node/processNoise", pN);
        nh.getParam("/ekf_node/sensorNoise", sN);

        processNoise *= pN;
        sensorNoise *= sN;

        sync.registerCallback(boost::bind(&Filter::callback, this, _1, _2));
    }

    ~Filter() {}

    void callback(const nav_msgs::OdometryConstPtr &odom_msg, const sensor_msgs::LaserScanConstPtr &scan_msg)
    {
        predict(odom_msg);

        bool detected = detectCircles(scan_msg);

        if (detected)
            correct();

        publishPose();
    }

    void predict(const nav_msgs::OdometryConstPtr &odom_msg)
    {
        tf2::Quaternion quat(
            odom_msg->pose.pose.orientation.x,
            odom_msg->pose.pose.orientation.y,
            odom_msg->pose.pose.orientation.z,
            odom_msg->pose.pose.orientation.w);

        double roll, pitch, theta;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, theta);

        // muPrev(0) = odom_msg->pose.pose.position.x;
        // muPrev(1) = odom_msg->pose.pose.position.y;
        // muPrev(2) = theta;

        double v = odom_msg->twist.twist.linear.x;
        double omega = odom_msg->twist.twist.angular.z;

        double dt = odom_msg->header.stamp.toSec() - oldTs;
        oldTs = odom_msg->header.stamp.toSec();

        if (fabs(omega) > 1e-5)
        {
            mm(0) = -v / omega * sin(theta) + v / omega * sin(theta + omega * dt);
            mm(1) = v / omega * cos(theta) - v / omega * cos(theta + omega * dt);
            mm(2) = omega * dt;
        }
        else
        {
            mm(0) = v * dt * cos(theta);
            mm(1) = v * dt * sin(theta);
            mm(2) = 0;
        }
        muPrev(3) = v;
        muPrev(4) = omega;
        muPred = muPrev + mm;
        sigmaPred = GPred * sigmaPrev * GPred.transpose() + processNoise;

        sigmaPub[0] = sigmaPred(0, 0);
        sigmaPub[1] = sigmaPred(0, 1);
        sigmaPub[5] = sigmaPred(0, 2);
        sigmaPub[6] = sigmaPred(1, 0);
        sigmaPub[7] = sigmaPred(1, 1);
        sigmaPub[11] = sigmaPred(1, 2);
        sigmaPub[30] = sigmaPred(2, 0);
        sigmaPub[31] = sigmaPred(2, 1);
        sigmaPub[35] = sigmaPred(2, 2);

        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "odom";
        pose_msg.pose.pose.position.x = muPred(0);
        pose_msg.pose.pose.position.y = muPred(1);
        pose_msg.pose.pose.position.z = 0.0;

        for (int i = 0; i < 36; ++i)
        {
            pose_msg.pose.covariance[i] = sigmaPub[i];
        }
    }

    bool detectCircles(const sensor_msgs::LaserScanConstPtr &scan_msg)
    {
        // Convert LaserScan to PointCloud2
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
        {
            if (std::isfinite(scan_msg->ranges[i]) && scan_msg->ranges[i] < 5.0)
            {
                float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
                pcl::PointXYZ point;
                point.x = scan_msg->ranges[i] * cos(angle);
                point.y = scan_msg->ranges[i] * sin(angle);
                point.z = 0;
                cloud->points.push_back(point);
            }
        }

        // Set up RANSAC for circle segmentation
        pcl::ModelCoefficients::Ptr coefficients_circle(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_circle(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CIRCLE2D);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);
        seg.setRadiusLimits(0.1, 0.7);
        seg.setInputCloud(cloud);
        seg.segment(*inliers_circle, *coefficients_circle);

        bool detected = false;

        if (inliers_circle->indices.size() == 0)
            ROS_WARN("Could not estimate a circle model for the given dataset.");
        else
        {
            detected = true;

            // Extract circle information
            double circleX = coefficients_circle->values[0];
            double circleY = coefficients_circle->values[1];
            double radius = coefficients_circle->values[2];

            double xWorld = circleX * cos(muPred(2)) - circleY * sin(muPred(2)) + muPred(0);
            double yWorld = circleX * sin(muPred(2)) + circleY * cos(muPred(2)) + muPred(1);

            double distance = sqrt(circleX * circleX + circleY * circleY);
            double angle = atan2(circleY, circleX);

            ROS_INFO("Detected circle at distance: %f, angle: %f radians", distance, angle);

            sm(0) = distance;
            sm(1) = angle;
        }

        return detected;
    }

    void correct()
    {
        Eigen::Vector2d delta;
        delta(0) = feature(0) - muPred(0);
        delta(1) = feature(1) - muPred(1);

        double q = delta.transpose() * delta;

        smPred(0) = std::sqrt(q);
        smPred(1) = std::atan2(delta(1), delta(0)) - muPred(2);

        H(0, 0) = delta(0) / smPred(0);
        H(0, 1) = delta(1) / smPred(0);
        H(1, 0) = -delta(1) / q;
        H(1, 1) = delta(0) / q;
        H(1, 2) = -1;

        KgainTemp = H * sigmaPred * H.transpose() + sensorNoise; // 2x5 * 5x5 * 5x2 + 2x2 = 2x2
        Kgain = sigmaPred * H.transpose() * KgainTemp.inverse(); // 5x5 * 5x2 * 2x2 = 5x2

        muCorr = muPred + Kgain * (sm - smPred); // 5x1 + 5x2 * (2x1 - 2x1) = 5x1
        muPrev = muCorr;

        sigmaCorr = (Identity - Kgain * H) * sigmaPred; // 5x5 - 5x2 * 2x5 = 5x5

        sigmaPub[0] = sigmaCorr(0, 0);
        sigmaPub[1] = sigmaCorr(0, 1);
        sigmaPub[5] = sigmaCorr(0, 2);
        sigmaPub[6] = sigmaCorr(1, 0);
        sigmaPub[7] = sigmaCorr(1, 1);
        sigmaPub[11] = sigmaCorr(1, 2);
        sigmaPub[30] = sigmaCorr(2, 0);
        sigmaPub[31] = sigmaCorr(2, 1);
        sigmaPub[35] = sigmaCorr(2, 2);

        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "odom";
        pose_msg.pose.pose.position.x = muCorr(0);
        pose_msg.pose.pose.position.y = muCorr(1);
        pose_msg.pose.pose.position.z = 0.0;

        for (int i = 0; i < 36; ++i)
        {
            pose_msg.pose.covariance[i] = sigmaPub[i];
        }

        sigmaPrev = sigmaCorr;
    }

    void publishPose()
    {
        pose_pub.publish(pose_msg);
    }

private:
    ros::NodeHandle nh;
    ros::NodeHandle pub;
    ros::Publisher pose_pub = pub.advertise<geometry_msgs::PoseWithCovarianceStamped>("my_corrected_pose", 1);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::LaserScan> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync;
    geometry_msgs::PoseWithCovarianceStamped pose_msg;

    Eigen::VectorXd muPrev = Eigen::VectorXd::Zero(5);
    Eigen::VectorXd muPred = Eigen::VectorXd::Zero(5);
    Eigen::VectorXd mm = Eigen::VectorXd::Zero(5);
    Eigen::VectorXd muCorr = Eigen::VectorXd::Zero(5);

    Eigen::MatrixXd GPred = Eigen::MatrixXd::Identity(5, 5);
    Eigen::MatrixXd sigmaPrev = Eigen::MatrixXd::Identity(5, 5);
    Eigen::MatrixXd sigmaPred = Eigen::MatrixXd::Zero(5, 5);
    Eigen::MatrixXd sigmaCorr = Eigen::MatrixXd::Zero(5, 5);
    Eigen::MatrixXd processNoise = Eigen::MatrixXd::Identity(5, 5);
    Eigen::MatrixXd sensorNoise = Eigen::MatrixXd::Identity(2, 2);

    Eigen::Vector2d feature = Eigen::Vector2d::Ones();
    Eigen::Vector2d sm = Eigen::Vector2d::Ones();
    Eigen::Vector2d smPred = Eigen::Vector2d::Zero();

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 5);
    Eigen::MatrixXd Kgain = Eigen::MatrixXd::Zero(5, 2);
    Eigen::MatrixXd KgainTemp = Eigen::MatrixXd::Zero(2, 2);
    Eigen::MatrixXd Identity = Eigen::MatrixXd::Identity(5, 5);

    double sigmaPub[36] = {0};

    double oldTs = 0;
    double pN;
    double sN;

    double xWorld = 0;
    double yWorld = 0;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf_localization_node");
    ros::NodeHandle nh;
    ros::NodeHandle pub;
    Filter ekfl(nh, pub);
    ros::spin();

    return 0;
}
