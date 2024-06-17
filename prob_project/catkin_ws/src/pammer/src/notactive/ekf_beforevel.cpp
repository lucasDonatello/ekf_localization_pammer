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

// sk learn
// pcl

class Filter
{
public:
    Filter(ros::NodeHandle &nh_, ros::NodeHandle &pub_) : nh(nh_), pub(pub_), odom_sub(nh, "odom", 1), scan_sub(nh, "scan", 1), sync(MySyncPolicy(10), odom_sub, scan_sub)
    {
        nh.getParam("/ekf_node/fx", feature(0));
        nh.getParam("/ekf_node/fy", feature(1));

        nh.getParam("/ekf_node/processNoise", pN);
        nh.getParam("/ekf_node/sensorNoise", sN);

        processNoise = pN * processNoise;
        sensorNoise = sN * sensorNoise;

        H(1, 2) = -1;

        sm(2) = 1;

        sync.registerCallback(boost::bind(&Filter::callback, this, _1, _2));
    }

    ~Filter() {}

    void callback(const nav_msgs::OdometryConstPtr &odom_msg, const sensor_msgs::LaserScanConstPtr &scan_msg)
    {
        predict(odom_msg);

        detectCircles(scan_msg);

        correct();
    }

    void predict(const nav_msgs::OdometryConstPtr &odom_msg)
    {
        tf2::Quaternion quat(
            odom_msg->pose.pose.orientation.x,
            odom_msg->pose.pose.orientation.y,
            odom_msg->pose.pose.orientation.z,
            odom_msg->pose.pose.orientation.w);

        // Convert quaternion to Euler angles
        double roll, pitch, theta;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, theta);

        muPrev(0) = odom_msg->pose.pose.position.x;
        muPrev(1) = odom_msg->pose.pose.position.y;
        muPrev(2) = theta;

        double v = odom_msg->twist.twist.linear.x;
        double omega = odom_msg->twist.twist.angular.z;

        double dt = odom_msg->header.stamp.toSec() - oldTs;
        oldTs = dt;

        // Avoid division by zero in the case of omega being very small
        if (fabs(omega) > 1e-5)
        {
            mm(0) = -v / omega * sin(theta) + v / omega * sin(theta + omega * dt);
            mm(1) = v / omega * cos(theta) - v / omega * cos(theta + omega * dt);
            mm(2) = omega * dt;

            GPred(0, 0) = 1.0;
            GPred(0, 1) = 0.0;
            GPred(0, 2) = v / omega * cos(theta) - v / omega * cos(theta + omega * dt);

            GPred(1, 0) = 0.0;
            GPred(1, 1) = 1.0;
            GPred(1, 2) = v / omega * sin(theta) - v / omega * sin(theta + omega * dt);

            GPred(2, 0) = 0.0;
            GPred(2, 1) = 0.0;
            GPred(2, 2) = 1.0;
        }
        else
        {
            mm(0) = v * dt * cos(theta);
            mm(1) = -v * dt * sin(theta);
            mm(2) = 0;

            GPred(0, 0) = 1.0;
            GPred(0, 1) = 0.0;
            GPred(0, 2) = v * dt * sin(theta);

            GPred(1, 0) = 0.0;
            GPred(1, 1) = 1.0;
            GPred(1, 2) = -v * dt * cos(theta);

            GPred(2, 0) = 0.0;
            GPred(2, 1) = 0.0;
            GPred(2, 2) = 1.0;
        }

        muPred = muPrev + mm;
        sigmaPred = GPred * sigmaPrev * GPred.transpose() + processNoise;
    }

    void detectCircles(const sensor_msgs::LaserScanConstPtr &scan_msg)
    {
        // Convert LaserScan to PointCloud2
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

        for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
        {
            if (scan_msg->ranges[i] < scan_msg->range_max && scan_msg->ranges[i] > scan_msg->range_min)
            {
                float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
                pcl::PointXYZ point;
                point.x = scan_msg->ranges[i] * cos(angle);
                point.y = scan_msg->ranges[i] * sin(angle);
                point.z = 0.0;
                cloud->points.push_back(point);
            }
        }

        // Detect circles
        pcl::ModelCoefficients::Ptr coefficients_circle(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_circle(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg_circle;
        seg_circle.setOptimizeCoefficients(true);
        seg_circle.setModelType(pcl::SACMODEL_CIRCLE2D);
        seg_circle.setMethodType(pcl::SAC_RANSAC);
        seg_circle.setDistanceThreshold(0.005);
        seg_circle.setInputCloud(cloud);
        seg_circle.segment(*inliers_circle, *coefficients_circle);

        if (inliers_circle->indices.size() == 0)
        {
            ROS_WARN("Could not estimate a circle model for the given dataset.");
            return;
        }

        double xCoord = coefficients_circle->values[0];
        double yCoord = coefficients_circle->values[1];
        double radius = coefficients_circle->values[2];

        xWorld = xCoord * cos(muPred(2)) - yCoord * sin(muPred(2)) + muPred(0);
        yWorld = xCoord * sin(muPred(2)) + yCoord * cos(muPred(2)) + muPred(1);

        // ROS_INFO("muX: %f - muY: %f - muTh: %f", muPred(0), muPred(1), muPred(2));

        // ROS_INFO("Circle detected: center (%f, %f), radius %f", xWorld, yWorld, radius);

        double distance = std::sqrt((xWorld - muPred(0)) * (xWorld - muPred(0)) + (yWorld - muPred(1)) * (yWorld - muPred(1)));
        double angle = std::atan2(yWorld - muPred(1), xWorld - muPred(0));

        // ROS_INFO("Distance to circle: %f, Angle to circle: %f", distance, angle);

        sm(0) = distance;
        sm(1) = angle;
    }

    void correct()
    {
        Eigen::Vector2d delta = Eigen::Vector2d::Zero();
        double q;

        delta(0) = feature(0) - muPred(0);
        delta(1) = feature(1) - muPred(1);

        q = delta.transpose() * delta;

        smPred(0) = std::sqrt(q);
        smPred(1) = std::atan2(delta(1), delta(0)) - muPred(2);
        smPred(2) = feature(2);

        H(0, 0) = std::sqrt(q) * delta(0);
        H(0, 1) = -std::sqrt(q) * delta(1);
        H(1, 0) = delta(1);
        H(1, 1) = delta(0);

        KgainTemp = H * sigmaPred * H.transpose() + sensorNoise;
        Kgain = sigmaPred * H.transpose() * KgainTemp.inverse();

        muCorr = muPred + Kgain * (sm - smPred);
        muPrev = muCorr;
        // --------
        sigmaCorr = (Identity - Kgain * H) * sigmaPred;
        // --------
        sigmaPrev = sigmaCorr;

        //  0,  1,   ,   ,   ,  5,   6,  7,   ,   ,    , 11,   ,   ,   ,   ,   ,    ,                                      30,  31 ,  ,    ,     ,  35
        // xx, xy, xz, xa, xb, xth, yx, yy, yz, ya, yb, yth, zx, zy, zz, za, zb, zth, a, a, a, a, a, a, b, b, b, b, b, b, thx, thy, thz, tha, thb, thth
        sigmaPub[0] = sigmaCorr(0, 0);
        sigmaPub[1] = sigmaCorr(0, 1);
        sigmaPub[5] = sigmaCorr(0, 2);
        sigmaPub[6] = sigmaCorr(1, 0);
        sigmaPub[7] = sigmaCorr(1, 1);
        sigmaPub[11] = sigmaCorr(1, 2);
        sigmaPub[30] = sigmaCorr(2, 0);
        sigmaPub[31] = sigmaCorr(2, 1);
        sigmaPub[35] = sigmaCorr(2, 2);

        geometry_msgs::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "odom";

        pose_msg.pose.pose.position.x = muCorr(0);
        pose_msg.pose.pose.position.y = muCorr(1);
        pose_msg.pose.pose.position.z = 0.0;

        for (int i = 0; i < 36; ++i)
        {
            pose_msg.pose.covariance[i] = sigmaPub[i];
        }

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
    Eigen::Vector3d muPrev = Eigen::Vector3d::Zero();
    Eigen::Vector3d muPred = Eigen::Vector3d::Zero();
    Eigen::Vector3d mm = Eigen::Vector3d::Zero();
    Eigen::Vector3d muCorr = Eigen::Vector3d::Zero();
    Eigen::Matrix3d GPred = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d sigmaPrev = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d sigmaPred = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d sigmaCorr = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d processNoise = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d sensorNoise = Eigen::Matrix3d::Identity();
    Eigen::Vector3d feature = Eigen::Vector3d::Ones();
    Eigen::Vector3d sm = Eigen::Vector3d::Ones();
    Eigen::Vector3d smPred = Eigen::Vector3d::Zero();
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d Kgain = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d KgainTemp = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d Identity = Eigen::Matrix3d::Identity();

    double sigmaPub[36] = {0};

    double oldTs = 0;
    double pN;
    double sN;

    double xWorld = 0;
    double yWorld = 0;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "synchronized_node");
    ros::NodeHandle nh;
    ros::NodeHandle pub;
    Filter ekfl(nh, pub);
    ros::spin();

    return 0;
}
