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
#include <pcl/console/print.h>

using namespace std;

// function to normalize angle
double normalizeAngle(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

class Filter
{
public:
    Filter(ros::NodeHandle &nh_, ros::NodeHandle &pub_) : nh(nh_), pub(pub_), odom_sub(nh, "odom", 1), scan_sub(nh, "scan", 1), sync(MySyncPolicy(10), odom_sub, scan_sub)
    {
        // reads landmark coordinates from ekf_param.yaml
        nh.getParam("/ekf_node/fx", landmark(0));
        nh.getParam("/ekf_node/fy", landmark(1));

        // reads noise values from ekf_param.yaml
        nh.getParam("/ekf_node/processNoise", pN);
        nh.getParam("/ekf_node/sensorNoise", sN);

        // multiplies the noise values with the corresponding diagonal matrices
        processNoise *= pN;
        sensorNoise *= sN;

        // reduces terminal messages
        pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

        // starts callback
        sync.registerCallback(boost::bind(&Filter::callback, this, _1, _2));
    }

    ~Filter() {}

    void callback(const nav_msgs::OdometryConstPtr &odom_msg, const sensor_msgs::LaserScanConstPtr &scan_msg)
    {
        // predicts pose with odometry data
        predict(odom_msg);

        // checks if circle was detected
        bool detected = detectCircles(scan_msg);

        // calls correction step when circle was detected
        if (detected)
            correct();

        // publishes predicted/corrected pose
        publishPose();

        count++;
    }

    void predict(const nav_msgs::OdometryConstPtr &odom_msg)
    {
        double roll, pitch, theta;

        // reads linear and angular speed from odometry
        double v = odom_msg->twist.twist.linear.x;
        double omega = odom_msg->twist.twist.angular.z;
        double dt = odom_msg->header.stamp.toSec() - oldTs;

        // transforms quaternions to euler angles
        tf2::Quaternion quat(
            odom_msg->pose.pose.orientation.x,
            odom_msg->pose.pose.orientation.y,
            odom_msg->pose.pose.orientation.z,
            odom_msg->pose.pose.orientation.w);

        tf2::Matrix3x3(quat).getRPY(roll, pitch, theta);

        // reads coordinates from odometry
        odomCycle(0) = odom_msg->pose.pose.position.x;
        odomCycle(1) = odom_msg->pose.pose.position.y;
        odomCycle(2) = normalizeAngle(theta);

        // initalizes the robot with odometry coordinates for first cycle
        if (count == 0)
        {
            muPrev(0) = odom_msg->pose.pose.position.x;
            muPrev(1) = odom_msg->pose.pose.position.y;
            muPrev(2) = normalizeAngle(theta);
        }
        else
            theta = normalizeAngle(muPrev(2));

        // reads timestamp
        oldTs = odom_msg->header.stamp.toSec();

        if (fabs(omega) > 0.001)
        {
            // motion model
            mm(0) = -v / omega * sin(theta) + v / omega * sin(theta + omega * dt);
            mm(1) = v / omega * cos(theta) - v / omega * cos(theta + omega * dt);
            mm(2) = omega * dt;

            // jacobian of motion model
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
            // simplification if omega (w) is too close to 0
            mm(0) = v * dt * cos(theta);
            mm(1) = -v * dt * sin(theta);
            mm(2) = 0.0;

            GPred(0, 0) = 1.0;
            GPred(0, 1) = 0.0;
            GPred(0, 2) = -v * dt * sin(theta);

            GPred(1, 0) = 0.0;
            GPred(1, 1) = 1.0;
            GPred(1, 2) = v * dt * cos(theta);

            GPred(2, 0) = 0.0;
            GPred(2, 1) = 0.0;
            GPred(2, 2) = 1.0;
        }

        // prediction calculation setting and setting calculation as old values for next cycle
        muPred = muPrev + mm;
        muPrev = muPred;
        sigmaPred = GPred * sigmaPrev * GPred.transpose() + processNoise;
        sigmaPrev = sigmaPred;

        // stores prediction values for covariance in format for PoseWithCovarianceStamped
        sigmaPub[0] = sigmaPred(0, 0);
        sigmaPub[7] = sigmaPred(1, 1);
        sigmaPub[35] = sigmaPred(2, 2);

        // calculates quaternion from prediction for PoseWithCovarianceStamped
        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, muPred(2));

        // writes to PoseWithCovarianceStamped to publish
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "map";
        pose_msg.pose.pose.position.x = muPred(0);
        pose_msg.pose.pose.position.y = muPred(1);
        pose_msg.pose.pose.position.z = 0.0;
        pose_msg.pose.pose.orientation.x = quaternion.x();
        pose_msg.pose.pose.orientation.y = quaternion.y();
        pose_msg.pose.pose.orientation.z = quaternion.z();
        pose_msg.pose.pose.orientation.w = quaternion.w();

        for (int i = 0; i < 36; ++i)
        {
            pose_msg.pose.covariance[i] = sigmaPub[i];
        }
    }

    bool detectCircles(const sensor_msgs::LaserScanConstPtr &scan_msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

        // reading LiDAR data (each single point) and storing it in PCL format
        // only points which are in range of 0 to 5 meters from robot, are considered
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

        // setup for PCL 2D circle detection
        pcl::ModelCoefficients::Ptr coefficients_circle(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_circle(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CIRCLE2D);
        seg.setMethodType(pcl::SAC_RANSAC); //choosing RANSAC
        seg.setDistanceThreshold(0.01);
        seg.setRadiusLimits(0.28, 0.32); // only considering detected circles with radius in between of 0.28 and 0.32 meters
        seg.setInputCloud(cloud);
        seg.segment(*inliers_circle, *coefficients_circle);

        bool detected = false;

        if (inliers_circle->indices.size() > 0)
        {
            // reading potential landmark from PCL segmentation
            double circleX = coefficients_circle->values[0];
            double circleY = coefficients_circle->values[1];
            double radius = coefficients_circle->values[2];

            // calculating detected circle coordinates in world frame
            double xWorld = circleX * cos(odomCycle(2)) - circleY * sin(odomCycle(2)) + odomCycle(0);
            double yWorld = circleX * sin(odomCycle(2)) + circleY * cos(odomCycle(2)) + odomCycle(1);

            // calculating difference between detected landmark and predefined landmark coordinates
            double diffX = xWorld - landmark(0);
            double diffY = yWorld - landmark(1);
            double diffTotal = std::sqrt(std::pow(diffX, 2) + std::pow(diffY, 2));

            // calculating distance and angle for corrections step
            double distance = sqrt(circleX * circleX + circleY * circleY);
            double angle = atan2(circleY, circleX);

            sm(0) = distance;
            sm(1) = angle;

            // checking if it is a potential landmark (within 1 meter of predefined coordinates)
            if (diffTotal <= 1.0)
                detected = true;
            else
                detected = false;
        }

        return detected;
    }

    void correct()
    {
        // correction step according to Thrun 2005 EKF algorithm with slight changes according to Chen 2012
        // clarified in paper "EKF Localization for Autonomous Mobile Robots using Multi-Sensor Data Fusion" by Pammer Lucas 2024
        Eigen::Vector2d delta;
        delta(0) = landmark(0) - muPred(0);
        delta(1) = landmark(1) - muPred(1);

        // double q = delta.transpose() * delta;

        double q = pow(delta(0), 2) + pow(delta(1), 2);

        smPred(0) = std::sqrt(q);
        smPred(1) = std::atan2(delta(1), delta(0)) - muPred(2);

        smPred(1) = normalizeAngle(smPred(1));

        H(0, 0) = delta(0) / std::sqrt(q);
        H(0, 1) = -delta(1) / std::sqrt(q);

        H(1, 0) = delta(1) / q;
        H(1, 1) = -delta(0) / q;

        // for robustness, 3x3 matrices are reduced to 2x2 for calculation
        // extracting relevant data
        sigmaPredShort(0, 0) = sigmaPred(0, 0);
        sigmaPredShort(0, 1) = sigmaPred(0, 1);
        sigmaPredShort(1, 0) = sigmaPred(1, 0);
        sigmaPredShort(1, 1) = sigmaPred(1, 1);

        KgainTemp = H * sigmaPredShort * H.transpose() + sensorNoise;
        Kgain = sigmaPredShort * H.transpose() * KgainTemp.inverse();

        muPredShort(0) = muPred(0);
        muPredShort(1) = muPred(1);

        // calculation of correction and converting 2x2 matrices back to 3x3
        muCorrShort = muPredShort + Kgain * (sm - smPred);
        muCorr(0) = muCorrShort(0);
        muCorr(1) = muCorrShort(1);
        muCorr(2) = normalizeAngle(muPrev(2));
        muPrev = muCorr;

        sigmaCorrShort = (Identity - Kgain * H) * sigmaPredShort;

        sigmaCorr = sigmaPred;

        sigmaCorr(0, 0) = sigmaCorrShort(0, 0);
        sigmaCorr(0, 1) = sigmaCorrShort(0, 1);
        sigmaCorr(1, 0) = sigmaCorrShort(1, 0);
        sigmaCorr(1, 1) = sigmaCorrShort(1, 1);

        // writes to PoseWithCovarianceStamped to publish
        // if correction is called, pose is overwritten from prediction step
        sigmaPub[0] = sigmaCorr(0, 0);
        sigmaPub[7] = sigmaCorr(1, 1);
        sigmaPub[35] = sigmaCorr(2, 2);

        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, muCorr(2));

        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "map";
        pose_msg.pose.pose.position.x = muCorr(0);
        pose_msg.pose.pose.position.y = muCorr(1);
        pose_msg.pose.pose.position.z = 0.0;
        pose_msg.pose.pose.orientation.x = quaternion.x();
        pose_msg.pose.pose.orientation.y = quaternion.y();
        pose_msg.pose.pose.orientation.z = quaternion.z();
        pose_msg.pose.pose.orientation.w = quaternion.w();

        for (int i = 0; i < 36; ++i)
        {
            pose_msg.pose.covariance[i] = sigmaPub[i];
        }

        // setting calculation to "previous" for next cycle
        sigmaPrev = sigmaCorr;
    }

    // funciton to call pose publication for calculated pose in prediction/correction
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

    Eigen::VectorXd muPrev = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd muPred = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd mm = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd muCorr = Eigen::VectorXd::Zero(3);

    Eigen::MatrixXd GPred = Eigen::MatrixXd::Identity(3, 3);
    Eigen::MatrixXd sigmaPrev = Eigen::MatrixXd::Identity(3, 3);
    Eigen::MatrixXd sigmaPred = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd sigmaCorr = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd processNoise = Eigen::MatrixXd::Identity(3, 3);
    Eigen::MatrixXd sensorNoise = Eigen::MatrixXd::Identity(2, 2);

    Eigen::VectorXd landmark = Eigen::VectorXd::Ones(3);
    Eigen::VectorXd sm = Eigen::VectorXd::Ones(2);
    Eigen::VectorXd smPred = Eigen::VectorXd::Ones(2);

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 2);
    Eigen::MatrixXd Kgain = Eigen::MatrixXd::Zero(2, 2);
    Eigen::MatrixXd KgainTemp = Eigen::MatrixXd::Zero(2, 2);
    Eigen::MatrixXd Identity = Eigen::MatrixXd::Identity(2, 2);

    Eigen::VectorXd odomCycle = Eigen::VectorXd::Zero(3);

    Eigen::MatrixXd sigmaCorrShort = Eigen::MatrixXd::Zero(2, 2);
    Eigen::MatrixXd sigmaPredShort = Eigen::MatrixXd::Zero(2, 2);
    Eigen::VectorXd muPredShort = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd muCorrShort = Eigen::VectorXd::Zero(2);

    double sigmaPub[36] = {0.0};

    double oldTs = 0.0;
    double pN;
    double sN;

    double xWorld = 0;
    double yWorld = 0;

    int count = 0;
};

int main(int argc, char **argv)
{
    // ROS initialization
    ros::init(argc, argv, "ekf_localization_node");
    ros::NodeHandle nh;
    ros::NodeHandle pub;

    // creating Filter object
    Filter ekfl(nh, pub);
    ros::spin();

    return 0;
}
